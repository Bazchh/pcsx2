// SPDX-FileCopyrightText: 2002-2026 PCSX2 Dev Team
// SPDX-License-Identifier: GPL-3.0+

#include "BuildVersion.h"
#include "Common.h"
#include "Host.h"
#include "Elfheader.h"
#include "SaveState.h"
#include "PINE.h"
#include "VMManager.h"
#include "vtlb.h"
#include "GS/Renderers/HW/GSSceneLayer.h"
#include "DebugTools/IAEmCall.h"
#include "DebugTools/IAEmWatch.h"
#include "common/Error.h"
#include "common/Threading.h"

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <span>
#include <sys/types.h>
#include <thread>

#include "fmt/format.h"

#if defined(_WIN32)
#define read_portable(a, b, c) (recv(a, (char*)b, c, 0))
#define write_portable(a, b, c) (send(a, (const char*)b, c, 0))
#define safe_close_portable(a) \
	do \
	{ \
		if ((a) >= 0) \
		{ \
			closesocket((a)); \
			(a) = INVALID_SOCKET; \
		} \
	} while (0)
#include "common/RedtapeWindows.h"
#include <WinSock2.h>
#elif defined(__linux__) || defined(__FreeBSD__)
#define read_portable(a, b, c) (read(a, b, c))
#define write_portable(a, b, c) (send(a, b, c, MSG_NOSIGNAL))
#define safe_close_portable(a) \
	do \
	{ \
		if ((a) >= 0) \
		{ \
			close((a)); \
			(a) = -1; \
		} \
	} while (0)
#include <sys/select.h>
#include <sys/socket.h>
#include <sys/un.h>
#include <unistd.h>
#else
#define read_portable(a, b, c) (read(a, b, c))
#define write_portable(a, b, c) (write(a, b, c))
#define safe_close_portable(a) \
	do \
	{ \
		if ((a) >= 0) \
		{ \
			close((a)); \
			(a) = -1; \
		} \
	} while (0)
#include <sys/select.h>
#include <sys/socket.h>
#include <sys/un.h>
#include <unistd.h>
#endif

#define PINE_EMULATOR_NAME "pcsx2"

#ifdef _WIN32

static bool InitializeWinsock()
{
	static bool initialized = false;
	if (initialized)
		return true;

	WSADATA wsa = {};
	if (WSAStartup(MAKEWORD(2, 2), &wsa) != 0)
		return false;

	initialized = true;
	std::atexit([]() { WSACleanup(); });
	return true;
}

#endif

namespace PINEServer
{
	static std::thread s_thread;
	static int s_slot;

#ifdef _WIN32
	// windows claim to have support for AF_UNIX sockets but that is a blatant lie,
	// their SDK won't even run their own examples, so we go on TCP sockets.
	static SOCKET s_sock = INVALID_SOCKET;
	// the message socket used in thread's accept().
	static SOCKET s_msgsock = INVALID_SOCKET;
#else
	// absolute path of the socket. Stored in XDG_RUNTIME_DIR, if unset /tmp
	static std::string s_socket_name;
	static int s_sock = -1;
	// the message socket used in thread's accept().
	static int s_msgsock = -1;
#endif

	// Whether the socket processing thread should stop executing/is stopped.
	static std::atomic_bool s_end{true};

	// IAEmPS2 extension.
	static constexpr u16 EXT_VERSION = 1;
	static constexpr u32 EXT_CAP_PAUSE = 1;
	static constexpr u32 EXT_CAP_SPEED = 2;
	static constexpr u32 EXT_CAP_SCENE = 8;
	static constexpr u32 EXT_CAP_WATCH = 16;
	static constexpr u32 EXT_CAP_CALL = 32;
	/** How long the server waits for a socket before checking the speed watchdog again. */
	static constexpr int EXT_WATCHDOG_POLL_MS = 100;
	/** Steady-clock instant, in ms, at which the limiter goes back to Nominal on its own. 0 means disarmed. */
	static std::atomic<s64> s_ext_speed_deadline_ms{0};

	static s64 ExtNowMs();
	/** Resumes normal speed on the CPU thread once the deadline has passed. Runs on the PINE thread. */
	static void ExtCheckSpeedWatchdog();
	/** Waits until the socket is readable (or accept-ready) or the timeout passes; false on timeout. */
	static bool ExtWaitReadable(decltype(s_sock) sock, int timeout_ms);

	/**
	 * Maximum memory used by an IPC message request.
	 * Equivalent to 50,000 Write64 requests.
	 */
#define MAX_IPC_SIZE 650000

	/**
	 * Maximum memory used by an IPC message reply.
	 * Equivalent to 50,000 Read64 replies.
	 */
#define MAX_IPC_RETURN_SIZE 450000

	/**
	 * IPC return buffer.
	 * A preallocated buffer used to store all IPC replies.
	 * to the size of 50.000 MsgWrite64 IPC calls.
	 */
	static std::vector<u8> s_ret_buffer;

	/**
	 * IPC messages buffer.
	 * A preallocated buffer used to store all IPC messages.
	 */
	static std::vector<u8> s_ipc_buffer;

	/**
	 * IPC Command messages opcodes.
	 * A list of possible operations possible by the IPC.
	 * Each one of them is what we call an "opcode" and is the first
	 * byte sent by the IPC to differentiate between commands.
	 */
	enum IPCCommand : unsigned char
	{
		MsgRead8 = 0, /**< Read 8 bit value to memory. */
		MsgRead16 = 1, /**< Read 16 bit value to memory. */
		MsgRead32 = 2, /**< Read 32 bit value to memory. */
		MsgRead64 = 3, /**< Read 64 bit value to memory. */
		MsgWrite8 = 4, /**< Write 8 bit value to memory. */
		MsgWrite16 = 5, /**< Write 16 bit value to memory. */
		MsgWrite32 = 6, /**< Write 32 bit value to memory. */
		MsgWrite64 = 7, /**< Write 64 bit value to memory. */
		MsgVersion = 8, /**< Returns PCSX2 version. */
		MsgSaveState = 9, /**< Saves a savestate. */
		MsgLoadState = 0xA, /**< Loads a savestate. */
		MsgTitle = 0xB, /**< Returns the game title. */
		MsgID = 0xC, /**< Returns the game ID. */
		MsgUUID = 0xD, /**< Returns the game UUID. */
		MsgGameVersion = 0xE, /**< Returns the game verion. */
		MsgStatus = 0xF, /**< Returns the emulator status. */
		// IAEmPS2 extension. Opcodes with the high bit set never collide with upstream additions.
		MsgExtCapabilities = 0x80, /**< Returns the extension version and capability mask. */
		MsgExtSetPaused = 0x81, /**< Pauses or resumes the VM, like the pause hotkey. */
		MsgExtSetSpeed = 0x82, /**< Sets the limiter mode and scalar, with an optional deadline. */
		MsgExtGetSpeed = 0x83, /**< Returns the limiter mode, target speed and remaining deadline. */
		// IAEmPS2 scene layer (phase 4).
		MsgExtSceneList = 0x84, /**< Returns the draw signatures of the last frame. */
		MsgExtSceneRule = 0x85, /**< Adds a scene rule (hide / inject quad) with a deadline. */
		MsgExtSceneClear = 0x86, /**< Removes one rule or all. */
		MsgExtSceneStats = 0x88, /**< Returns rule counters and layer cost. */
		// IAEmPS2 memory watch (phase 2).
		MsgExtWatchSet = 0x89, /**< Watches one address range for reads/writes, recording hits by pc. */
		MsgExtWatchRead = 0x8A, /**< Returns the hit table, optionally resetting it. */
		MsgExtWatchClear = 0x8B, /**< Removes the watch. */
		// IAEmPS2 remote call (phase 2).
		MsgExtCall = 0x8C, /**< Runs one function of the game on the EE and returns v0/v1. */
		MsgUnimplemented = 0xFF /**< Unimplemented IPC message. */
	};

	/**
	 * Emulator status enum.
	 * A list of possible emulator statuses.
	 */
	enum EmuStatus : uint32_t
	{
		Running = 0, /**< Game is running */
		Paused = 1, /**< Game is paused */
		Shutdown = 2 /**< Game is shutdown */
	};

	/**
	 * IPC message buffer.
	 * A list of all needed fields to store an IPC message.
	 */
	struct IPCBuffer
	{
		int size; /**< Size of the buffer. */
		std::vector<u8> buffer; /**< Buffer. */
	};

	/**
	 * IPC result codes.
	 * A list of possible result codes the IPC can send back.
	 * Each one of them is what we call an "opcode" or "tag" and is the
	 * first byte sent by the IPC to differentiate between results.
	 */
	enum IPCResult : unsigned char
	{
		IPC_OK = 0, /**< IPC command successfully completed. */
		IPC_FAIL = 0xFF /**< IPC command failed to complete. */
	};

	// Thread used to relay IPC commands.
	void MainLoop();
	void ClientLoop();

	/**
	 * Internal function, Parses an IPC command.
	 * buf: buffer containing the IPC command.
	 * buf_size: size of the buffer announced.
	 * ret_buffer: buffer that will be used to send the reply.
	 * return value: IPCBuffer containing a buffer with the result
	 *               of the command and its size.
	 */
	static IPCBuffer ParseCommand(std::span<u8> buf, std::vector<u8>& ret_buffer, u32 buf_size);

	/**
	 * Formats an IPC buffer
	 * ret_buffer: return buffer to use.
	 * size: size of the IPC buffer.
	 * return value: buffer containing the status code allocated of size
	 */
	static std::vector<u8>& MakeOkIPC(std::vector<u8>& ret_buffer, uint32_t size);
	static std::vector<u8>& MakeFailIPC(std::vector<u8>& ret_buffer, uint32_t size);

	/**
	 * Initializes an open socket for IPC communication.
	 */
	bool AcceptClient();

	/**
	 * Converts a primitive value to bytes in little endian
	 * res_vector: the vector to modify
	 * res: the value to convert
	 * i: where to insert it into the vector
	 * NB: implicitely inlined
	 */
	template <typename T>
	static void ToResultVector(std::vector<u8>& res_vector, T res, int i)
	{
		memcpy(&res_vector[i], (char*)&res, sizeof(T));
	}

	/**
	 * Converts bytes in little endian to a primitive value
	 * span: the span to convert
	 * i: where to load it from the span
	 * return value: the converted value
	 * NB: implicitely inlined
	 */
	template <typename T>
	static T FromSpan(std::span<u8> span, int i)
	{
		return *(T*)(&span[i]);
	}

	/**
	 * Ensures an IPC message isn't too big.
	 * return value: false if checks failed, true otherwise.
	 */
	static inline bool SafetyChecks(u32 command_len, int command_size, u32 reply_len, int reply_size = 0, u32 buf_size = MAX_IPC_SIZE - 1)
	{
		return !((command_len + command_size) > buf_size ||
				 (reply_len + reply_size) >= MAX_IPC_RETURN_SIZE);
	}
} // namespace PINEServer

s64 PINEServer::ExtNowMs()
{
	return std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now().time_since_epoch()).count();
}

void PINEServer::ExtCheckSpeedWatchdog()
{
	s64 deadline = s_ext_speed_deadline_ms.load(std::memory_order_acquire);
	if (deadline == 0 || ExtNowMs() < deadline)
		return;
	// Whoever disarms it is the one that resumes, so the resume runs exactly once.
	if (!s_ext_speed_deadline_ms.compare_exchange_strong(deadline, 0))
		return;
	Host::RunOnCPUThread([] {
		if (!VMManager::HasValidVM())
			return;
		VMManager::SetLimiterMode(LimiterModeType::Nominal);
		VMManager::UpdateTargetSpeed();
	});
}

bool PINEServer::ExtWaitReadable(decltype(s_sock) sock, int timeout_ms)
{
	fd_set set;
	FD_ZERO(&set);
	FD_SET(sock, &set);
	timeval tv;
	tv.tv_sec = timeout_ms / 1000;
	tv.tv_usec = (timeout_ms % 1000) * 1000;
#ifdef _WIN32
	const int nfds = 0; // ignored by Winsock
#else
	const int nfds = static_cast<int>(sock) + 1;
#endif
	// An error (-1) also returns true: the blocking call that follows reports it the usual way.
	return select(nfds, &set, nullptr, nullptr, &tv) != 0;
}

bool PINEServer::Initialize(int slot)
{
	s_end.store(false, std::memory_order_release);
	s_slot = slot;

#ifdef _WIN32
	if (!InitializeWinsock())
	{
		Console.WriteLn(Color_Red, "PINE: Cannot initialize winsock! Shutting down...");
		Deinitialize();
		return false;
	}

	s_sock = socket(AF_INET, SOCK_STREAM, 0);
	if ((s_sock == INVALID_SOCKET) || slot > 65536)
	{
		Console.WriteLn(Color_Red, "PINE: Cannot open socket! Shutting down...");
		Deinitialize();
		return false;
	}

	sockaddr_in server = {};
	server.sin_family = AF_INET;
	server.sin_addr.s_addr = htonl(INADDR_LOOPBACK); // localhost only
	server.sin_port = htons(slot);

	if (bind(s_sock, (struct sockaddr*)&server, sizeof(server)) == SOCKET_ERROR)
	{
		Console.WriteLn(Color_Red, "PINE: Error while binding to socket! Shutting down...");
		Deinitialize();
		return false;
	}

#else
	char* runtime_dir = nullptr;
#ifdef __APPLE__
	runtime_dir = std::getenv("TMPDIR");
#else
	runtime_dir = std::getenv("XDG_RUNTIME_DIR");
#endif
	// fallback in case macOS or other OSes don't implement the XDG base
	// spec
	if (runtime_dir == nullptr)
		s_socket_name = "/tmp/" PINE_EMULATOR_NAME ".sock";
	else
	{
		s_socket_name = runtime_dir;
		s_socket_name += "/" PINE_EMULATOR_NAME ".sock";
	}

	if (slot != PINE_DEFAULT_SLOT)
		s_socket_name += "." + std::to_string(slot);

	struct sockaddr_un server;

	s_sock = socket(AF_UNIX, SOCK_STREAM, 0);
	if (s_sock < 0)
	{
		Console.WriteLn(Color_Red, "PINE: Cannot open socket! Shutting down...");
		Deinitialize();
		return false;
	}
	server.sun_family = AF_UNIX;
	StringUtil::Strlcpy(server.sun_path, s_socket_name, sizeof(server.sun_path));

	// we unlink the socket so that when releasing this thread the socket gets
	// freed even if we didn't close correctly the loop
	unlink(s_socket_name.c_str());
	if (bind(s_sock, (struct sockaddr*)&server, sizeof(struct sockaddr_un)))
	{
		Console.WriteLn(Color_Red, "PINE: Error while binding to socket! Shutting down...");
		Deinitialize();
		return false;
	}
#endif

	// maximum queue of 4096 commands before refusing, approximated to the
	// nearest legal value. We do not use SOMAXCONN as windows have this idea
	// that a "reasonable" value is 5, which is not.
	if (listen(s_sock, 4096))
	{
		Console.WriteLn(Color_Red, "PINE: Cannot listen for connections! Shutting down...");
		Deinitialize();
		return false;
	}

	// we allocate once buffers to not have to do mallocs for each IPC
	// request, as malloc is expansive when we optimize for µs.
	s_ret_buffer.resize(MAX_IPC_RETURN_SIZE);
	s_ipc_buffer.resize(MAX_IPC_SIZE);

	// we start the thread
	s_thread = std::thread(&PINEServer::MainLoop);

	return true;
}

bool PINEServer::IsInitialized()
{
	return !s_end.load(std::memory_order_acquire);
}

int PINEServer::GetSlot()
{
	return s_slot;
}

std::vector<u8>& PINEServer::MakeOkIPC(std::vector<u8>& ret_buffer, uint32_t size = 5)
{
	ToResultVector<uint32_t>(ret_buffer, size, 0);
	ret_buffer[4] = IPC_OK;
	return ret_buffer;
}

std::vector<u8>& PINEServer::MakeFailIPC(std::vector<u8>& ret_buffer, uint32_t size = 5)
{
	ToResultVector<uint32_t>(ret_buffer, size, 0);
	ret_buffer[4] = IPC_FAIL;
	return ret_buffer;
}

bool PINEServer::AcceptClient()
{
	s_msgsock = accept(s_sock, 0, 0);
	if (s_msgsock < 0)
	{
		// everything else is non recoverable in our scope
		// we also mark as recoverable socket errors where it would block a
		// non blocking socket, even though our socket is blocking, in case
		// we ever have to implement a non blocking socket.
#ifdef _WIN32
		const int errno_w = WSAGetLastError();
		if (!(errno_w == WSAECONNRESET || errno_w == WSAEINTR || errno_w == WSAEINPROGRESS || errno_w == WSAEMFILE || errno_w == WSAEWOULDBLOCK) && s_sock != INVALID_SOCKET)
			Console.Error("PINE: accept() returned error %d", errno_w);
#else
		if (!(errno == ECONNABORTED || errno == EINTR || errno == EAGAIN || errno == EWOULDBLOCK) && s_sock >= 0)
			Console.Error("PINE: accept() returned error %d", errno);
#endif

		return false;
	}

#ifdef __APPLE__
	int nosigpipe = 1;
	setsockopt(s_msgsock, SOL_SOCKET, SO_NOSIGPIPE, &nosigpipe, sizeof(nosigpipe));
#endif

	// Gross C-style cast, but SOCKET is a handle on Windows.
	Console.WriteLn("PINE: New client with FD %d connected.", (int)s_msgsock);
	return true;
}

void PINEServer::MainLoop()
{
	Threading::SetNameOfCurrentThread("PINE Server");

	while (!s_end.load(std::memory_order_acquire))
	{
		// The watchdog has to fire with no client connected too, so never block in accept() for long.
		if (!ExtWaitReadable(s_sock, EXT_WATCHDOG_POLL_MS))
		{
			ExtCheckSpeedWatchdog();
			IAEmWatch::CheckDeadline();
			continue;
		}
		if (!AcceptClient())
			continue;

		ClientLoop();

		Console.WriteLn("PINE: Client disconnected.");
		safe_close_portable(s_msgsock);
	}
}

void PINEServer::ClientLoop()
{
	while (!s_end.load(std::memory_order_acquire))
	{
		// either int or ssize_t depending on the platform, so we have to
		// use a bunch of auto
		auto receive_length = 0;
		auto end_length = 4;
		const std::span<u8> ipc_buffer_span(s_ipc_buffer);

		// while we haven't received the entire packet, maybe due to
		// socket datagram splittage, we continue to read
		while (receive_length < end_length)
		{
			if (!ExtWaitReadable(s_msgsock, EXT_WATCHDOG_POLL_MS))
			{
				ExtCheckSpeedWatchdog();
				IAEmWatch::CheckDeadline();
				if (s_end.load(std::memory_order_acquire))
					return;
				continue;
			}
			const auto tmp_length = read_portable(s_msgsock, &ipc_buffer_span[receive_length], MAX_IPC_SIZE - receive_length);

			// we recreate the socket if an error happens
			if (tmp_length <= 0)
				return;

			receive_length += tmp_length;

			// if we got at least the final size then update
			if (end_length == 4 && receive_length >= 4)
			{
				end_length = FromSpan<u32>(ipc_buffer_span, 0);
				// we'd like to avoid a client trying to do OOB
				if (end_length > MAX_IPC_SIZE || end_length < 4)
				{
					receive_length = 0;
					break;
				}
			}
		}
		PINEServer::IPCBuffer res;

		// we remove 4 bytes to get the message size out of the IPC command
		// size in ParseCommand.
		// also, if we got a failed command, let's reset the state so we don't
		// end up deadlocking by getting out of sync, eg when a client
		// disconnects
		if (receive_length != 0)
		{
			res = ParseCommand(ipc_buffer_span.subspan(4), s_ret_buffer, (u32)end_length - 4);

			// if we cannot send back our answer restart the socket
			if (write_portable(s_msgsock, res.buffer.data(), res.size) < 0)
				return;
		}
	}
}

void PINEServer::Deinitialize()
{
	s_end.store(true, std::memory_order_release);

#ifndef _WIN32
	if (!s_socket_name.empty())
	{
		unlink(s_socket_name.c_str());
		s_socket_name = {};
	}
#endif

	// shutdown() is needed, otherwise accept() will still block.
#ifdef _WIN32
	if (s_sock != INVALID_SOCKET)
		shutdown(s_sock, SD_BOTH);
#else
	if (s_sock >= 0)
		shutdown(s_sock, SHUT_RDWR);
#endif

	safe_close_portable(s_sock);
	safe_close_portable(s_msgsock);

	if (s_thread.joinable())
		s_thread.join();
}

PINEServer::IPCBuffer PINEServer::ParseCommand(std::span<u8> buf, std::vector<u8>& ret_buffer, u32 buf_size)
{
	u32 ret_cnt = 5;
	u32 buf_cnt = 0;

	while (buf_cnt < buf_size)
	{
		if (!SafetyChecks(buf_cnt, 1, ret_cnt, 0, buf_size)) [[unlikely]]
			return IPCBuffer{5, MakeFailIPC(ret_buffer)};
		buf_cnt++;
		// example IPC messages: MsgRead/Write
		// refer to the client doc for more info on the format
		//         IPC Message event (1 byte)
		//         |  Memory address (4 byte)
		//         |  |           argument (VLE)
		//         |  |           |
		// format: XX YY YY YY YY ZZ ZZ ZZ ZZ
		//        reply code: 00 = OK, FF = NOT OK
		//        |  return value (VLE)
		//        |  |
		// reply: XX ZZ ZZ ZZ ZZ
		switch ((IPCCommand)buf[buf_cnt - 1])
		{
			case MsgRead8:
			{
				if (!VMManager::HasValidVM())
					goto error;
				if (!SafetyChecks(buf_cnt, 4, ret_cnt, 1, buf_size)) [[unlikely]]
					goto error;
				const u32 a = FromSpan<u32>(buf, buf_cnt);
				const u8 res = vtlb_ramRead<mem8_t>(a);
				ToResultVector(ret_buffer, res, ret_cnt);
				ret_cnt += 1;
				buf_cnt += 4;
				break;
			}
			case MsgRead16:
			{
				if (!VMManager::HasValidVM())
					goto error;
				if (!SafetyChecks(buf_cnt, 4, ret_cnt, 2, buf_size)) [[unlikely]]
					goto error;
				const u32 a = FromSpan<u32>(buf, buf_cnt);
				const u16 res = vtlb_ramRead<mem16_t>(a);
				ToResultVector(ret_buffer, res, ret_cnt);
				ret_cnt += 2;
				buf_cnt += 4;
				break;
			}
			case MsgRead32:
			{
				if (!VMManager::HasValidVM())
					goto error;
				if (!SafetyChecks(buf_cnt, 4, ret_cnt, 4, buf_size)) [[unlikely]]
					goto error;
				const u32 a = FromSpan<u32>(buf, buf_cnt);
				const u32 res = vtlb_ramRead<mem32_t>(a);
				ToResultVector(ret_buffer, res, ret_cnt);
				ret_cnt += 4;
				buf_cnt += 4;
				break;
			}
			case MsgRead64:
			{
				if (!VMManager::HasValidVM())
					goto error;
				if (!SafetyChecks(buf_cnt, 4, ret_cnt, 8, buf_size)) [[unlikely]]
					goto error;
				const u32 a = FromSpan<u32>(buf, buf_cnt);
				const u64 res = vtlb_ramRead<mem64_t>(a);
				ToResultVector(ret_buffer, res, ret_cnt);
				ret_cnt += 8;
				buf_cnt += 4;
				break;
			}
			case MsgWrite8:
			{
				if (!VMManager::HasValidVM())
					goto error;
				if (!SafetyChecks(buf_cnt, 1 + 4, ret_cnt, 0, buf_size)) [[unlikely]]
					goto error;
				const u32 a = FromSpan<u32>(buf, buf_cnt);
				vtlb_ramWrite<mem8_t>(a, FromSpan<u8>(buf, buf_cnt + 4));
				buf_cnt += 5;
				break;
			}
			case MsgWrite16:
			{
				if (!VMManager::HasValidVM())
					goto error;
				if (!SafetyChecks(buf_cnt, 2 + 4, ret_cnt, 0, buf_size)) [[unlikely]]
					goto error;
				const u32 a = FromSpan<u32>(buf, buf_cnt);
				vtlb_ramWrite<mem16_t>(a, FromSpan<u16>(buf, buf_cnt + 4));
				buf_cnt += 6;
				break;
			}
			case MsgWrite32:
			{
				if (!VMManager::HasValidVM())
					goto error;
				if (!SafetyChecks(buf_cnt, 4 + 4, ret_cnt, 0, buf_size)) [[unlikely]]
					goto error;
				const u32 a = FromSpan<u32>(buf, buf_cnt);
				vtlb_ramWrite<mem32_t>(a, FromSpan<u32>(buf, buf_cnt + 4));
				buf_cnt += 8;
				break;
			}
			case MsgWrite64:
			{
				if (!VMManager::HasValidVM())
					goto error;
				if (!SafetyChecks(buf_cnt, 8 + 4, ret_cnt, 0, buf_size)) [[unlikely]]
					goto error;
				const u32 a = FromSpan<u32>(buf, buf_cnt);
				vtlb_ramWrite<mem64_t>(a, FromSpan<u64>(buf, buf_cnt + 4));
				buf_cnt += 12;
				break;
			}
			case MsgVersion:
			{
				// The suffix tells clients this build carries the extension; the "PCSX2 " prefix is unchanged.
				const std::string version = fmt::format("PCSX2 {} +iaemps2-ext{}", BuildVersion::GitRev, EXT_VERSION);
				const u32 size = version.size() + 1;
				if (!SafetyChecks(buf_cnt, 0, ret_cnt, size + 4, buf_size)) [[unlikely]]
					goto error;
				ToResultVector(ret_buffer, size, ret_cnt);
				ret_cnt += 4;
				memcpy(&ret_buffer[ret_cnt], version.c_str(), size);
				ret_cnt += size;
				break;
			}
			case MsgSaveState:
			{
				if (!VMManager::HasValidVM())
					goto error;
				if (!SafetyChecks(buf_cnt, 1, ret_cnt, 0, buf_size)) [[unlikely]]
					goto error;
				Host::RunOnCPUThread([slot = FromSpan<u8>(buf, buf_cnt)] {
					VMManager::SaveStateToSlot(slot, true, [slot](const std::string& error) {
						SaveState_ReportSaveErrorOSD(error, slot);
					});
				});
				buf_cnt += 1;
				break;
			}
			case MsgLoadState:
			{
				if (!VMManager::HasValidVM())
					goto error;
				if (!SafetyChecks(buf_cnt, 1, ret_cnt, 0, buf_size)) [[unlikely]]
					goto error;
				Host::RunOnCPUThread([slot = FromSpan<u8>(buf, buf_cnt)] {
					Error state_error;
					if (!VMManager::LoadStateFromSlot(slot, false, &state_error))
						SaveState_ReportLoadErrorOSD(state_error.GetDescription(), slot, false);
				});
				buf_cnt += 1;
				break;
			}
			case MsgTitle:
			{
				if (!VMManager::HasValidVM())
					goto error;
				const std::string gameName = VMManager::GetTitle(false);
				const u32 size = gameName.size() + 1;
				if (!SafetyChecks(buf_cnt, 0, ret_cnt, size + 4, buf_size)) [[unlikely]]
					goto error;
				ToResultVector(ret_buffer, size, ret_cnt);
				ret_cnt += 4;
				memcpy(&ret_buffer[ret_cnt], gameName.c_str(), size);
				ret_cnt += size;
				break;
			}
			case MsgID:
			{
				if (!VMManager::HasValidVM())
					goto error;
				const std::string gameSerial = VMManager::GetDiscSerial();
				const u32 size = gameSerial.size() + 1;
				if (!SafetyChecks(buf_cnt, 0, ret_cnt, size + 4, buf_size)) [[unlikely]]
					goto error;
				ToResultVector(ret_buffer, size, ret_cnt);
				ret_cnt += 4;
				memcpy(&ret_buffer[ret_cnt], gameSerial.c_str(), size);
				ret_cnt += size;
				break;
			}
			case MsgUUID:
			{
				if (!VMManager::HasValidVM())
					goto error;
				const std::string crc = fmt::format("{:08x}", VMManager::GetDiscCRC());
				const u32 size = crc.size() + 1;
				if (!SafetyChecks(buf_cnt, 0, ret_cnt, size + 4, buf_size)) [[unlikely]]
					goto error;
				ToResultVector(ret_buffer, size, ret_cnt);
				ret_cnt += 4;
				memcpy(&ret_buffer[ret_cnt], crc.c_str(), size);
				ret_cnt += size;
				break;
			}
			case MsgGameVersion:
			{
				if (!VMManager::HasValidVM())
					goto error;

				const std::string ElfVersion = VMManager::GetDiscVersion();
				const u32 size = ElfVersion.size() + 1;
				if (!SafetyChecks(buf_cnt, 0, ret_cnt, size + 4, buf_size)) [[unlikely]]
					goto error;
				ToResultVector(ret_buffer, size, ret_cnt);
				ret_cnt += 4;
				memcpy(&ret_buffer[ret_cnt], ElfVersion.c_str(), size);
				ret_cnt += size;
				break;
			}
			case MsgStatus:
			{
				if (!SafetyChecks(buf_cnt, 0, ret_cnt, 4, buf_size)) [[unlikely]]
					goto error;
				EmuStatus status;

				switch (VMManager::GetState())
				{
					case VMState::Running:
						status = EmuStatus::Running;
						break;
					case VMState::Paused:
						status = EmuStatus::Paused;
						break;
					default:
						status = EmuStatus::Shutdown;
						break;
				}

				ToResultVector(ret_buffer, status, ret_cnt);
				ret_cnt += 4;
				break;
			}
			case MsgExtCapabilities:
			{
				// Reply: u16 extension version, u32 capability mask. Fails on upstream builds (default case).
				if (!SafetyChecks(buf_cnt, 0, ret_cnt, 2 + 4, buf_size)) [[unlikely]]
					goto error;
				ToResultVector(ret_buffer, EXT_VERSION, ret_cnt);
				ToResultVector(ret_buffer, static_cast<u32>(EXT_CAP_PAUSE | EXT_CAP_SPEED | EXT_CAP_SCENE | EXT_CAP_WATCH | EXT_CAP_CALL), ret_cnt + 2);
				ret_cnt += 6;
				break;
			}
			case MsgExtSetPaused:
			{
				// Argument: u8 (1 pause, 0 resume). Confirmation comes through MsgStatus.
				if (!VMManager::HasValidVM())
					goto error;
				if (!SafetyChecks(buf_cnt, 1, ret_cnt, 0, buf_size)) [[unlikely]]
					goto error;
				Host::RunOnCPUThread([paused = (FromSpan<u8>(buf, buf_cnt) != 0)] {
					VMManager::SetPaused(paused);
				});
				buf_cnt += 1;
				break;
			}
			case MsgExtSetSpeed:
			{
				// Arguments: u8 mode (0 nominal, 1 slomo, 2 turbo), f32 scalar, u32 duration in ms.
				// Mode 0 ignores the scalar and disarms the watchdog. Duration 0 keeps the mode until told otherwise.
				if (!VMManager::HasValidVM())
					goto error;
				if (!SafetyChecks(buf_cnt, 1 + 4 + 4, ret_cnt, 0, buf_size)) [[unlikely]]
					goto error;
				const u8 mode = FromSpan<u8>(buf, buf_cnt);
				const float scalar = FromSpan<float>(buf, buf_cnt + 1);
				const u32 duration_ms = FromSpan<u32>(buf, buf_cnt + 5);
				// A scalar of 0 would mean unlimited speed, the opposite of what a slow-motion caller wants.
				if (mode > 2 || (mode != 0 && !(std::isfinite(scalar) && scalar > 0.0f)))
					goto error;
				const s64 deadline = (mode != 0 && duration_ms > 0) ? (ExtNowMs() + duration_ms) : 0;
				s_ext_speed_deadline_ms.store(deadline, std::memory_order_release);
				Host::RunOnCPUThread([mode, scalar] {
					if (!VMManager::HasValidVM())
						return;
					LimiterModeType limiter = LimiterModeType::Nominal;
					if (mode == 1)
					{
						EmuConfig.EmulationSpeed.SlomoScalar = scalar;
						limiter = LimiterModeType::Slomo;
					}
					else if (mode == 2)
					{
						EmuConfig.EmulationSpeed.TurboScalar = scalar;
						limiter = LimiterModeType::Turbo;
					}
					VMManager::SetLimiterMode(limiter);
					// SetLimiterMode() returns early when the mode is unchanged, but the scalar may have changed.
					VMManager::UpdateTargetSpeed();
				});
				buf_cnt += 9;
				break;
			}
			case MsgExtGetSpeed:
			{
				// Reply: u8 mode (3 = unlimited, set by the player's hotkey), f32 target speed, u32 ms left on the watchdog.
				if (!SafetyChecks(buf_cnt, 0, ret_cnt, 1 + 4 + 4, buf_size)) [[unlikely]]
					goto error;
				u8 mode = 0;
				switch (VMManager::GetLimiterMode())
				{
					case LimiterModeType::Slomo:
						mode = 1;
						break;
					case LimiterModeType::Turbo:
						mode = 2;
						break;
					case LimiterModeType::Unlimited:
						mode = 3;
						break;
					default:
						mode = 0;
						break;
				}
				const s64 deadline = s_ext_speed_deadline_ms.load(std::memory_order_acquire);
				const s64 remaining = (deadline == 0) ? 0 : std::max<s64>(deadline - ExtNowMs(), 0);
				ToResultVector(ret_buffer, mode, ret_cnt);
				ToResultVector(ret_buffer, VMManager::GetTargetSpeed(), ret_cnt + 1);
				ToResultVector(ret_buffer, static_cast<u32>(remaining), ret_cnt + 5);
				ret_cnt += 9;
				break;
			}
			case MsgExtSceneList:
			{
				// Reply: u32 frame, u32 n, then n x 40-byte signatures (capped so the reply fits).
				u32 frame_no = 0;
				const std::vector<GSSceneLayer::Signature> list = GSSceneLayer::LastFrame(&frame_no);
				const u32 n = std::min<u32>(static_cast<u32>(list.size()), 8000);
				if (!SafetyChecks(buf_cnt, 0, ret_cnt, 8 + n * 40, buf_size)) [[unlikely]]
					goto error;
				ToResultVector(ret_buffer, frame_no, ret_cnt);
				ToResultVector(ret_buffer, n, ret_cnt + 4);
				ret_cnt += 8;
				if (n > 0)
				{
					memcpy(&ret_buffer[ret_cnt], list.data(), n * 40);
					ret_cnt += n * 40;
				}
				break;
			}
			case MsgExtSceneRule:
			{
				// Arguments: u8 kind, u64 tex hash, u64 shape hash, 4 x s16 rect, u32 z, u32 rgba, u32 duration ms, u32 after_nth. Reply: u32 id.
				if (!SafetyChecks(buf_cnt, 1 + 8 + 8 + 8 + 4 + 4 + 4 + 4, ret_cnt, 4, buf_size)) [[unlikely]]
					goto error;
				GSSceneLayer::Rule rule = {};
				rule.kind = FromSpan<u8>(buf, buf_cnt);
				rule.tex_hash = FromSpan<u64>(buf, buf_cnt + 1);
				rule.shape_hash = FromSpan<u64>(buf, buf_cnt + 9);
				rule.x0 = FromSpan<s16>(buf, buf_cnt + 17);
				rule.y0 = FromSpan<s16>(buf, buf_cnt + 19);
				rule.x1 = FromSpan<s16>(buf, buf_cnt + 21);
				rule.y1 = FromSpan<s16>(buf, buf_cnt + 23);
				rule.z = FromSpan<u32>(buf, buf_cnt + 25);
				rule.rgba = FromSpan<u32>(buf, buf_cnt + 29);
				const u32 duration_ms = FromSpan<u32>(buf, buf_cnt + 33);
				rule.after_nth = FromSpan<u32>(buf, buf_cnt + 37);
				if (rule.kind != GSSceneLayer::Hide && rule.kind != GSSceneLayer::InjectQuad)
					goto error;
				const u32 id = GSSceneLayer::AddRule(rule, duration_ms);
				ToResultVector(ret_buffer, id, ret_cnt);
				ret_cnt += 4;
				buf_cnt += 41;
				break;
			}
			case MsgExtSceneClear:
			{
				if (!SafetyChecks(buf_cnt, 4, ret_cnt, 0, buf_size)) [[unlikely]]
					goto error;
				GSSceneLayer::ClearRules(FromSpan<u32>(buf, buf_cnt));
				buf_cnt += 4;
				break;
			}
			case MsgExtSceneStats:
			{
				// Argument: u32 rule id (0 = layer only). Reply: u32 exists, u32 frames_total, u32 seen, u32 applied,
				// u32 layer_frames, u64 ns_total, u64 ns_last_frame, u32 draws_last, u32 qualifying_last, u32 hashed_last.
				if (!SafetyChecks(buf_cnt, 4, ret_cnt, 4 * 5 + 8 * 2 + 4 * 3, buf_size)) [[unlikely]]
					goto error;
				const GSSceneLayer::RuleStats rs = GSSceneLayer::GetRuleStats(FromSpan<u32>(buf, buf_cnt));
				const GSSceneLayer::LayerStats ls = GSSceneLayer::GetLayerStats();
				ToResultVector(ret_buffer, static_cast<u32>(rs.exists ? 1 : 0), ret_cnt);
				ToResultVector(ret_buffer, rs.frames_total, ret_cnt + 4);
				ToResultVector(ret_buffer, rs.frames_seen, ret_cnt + 8);
				ToResultVector(ret_buffer, rs.frames_applied, ret_cnt + 12);
				ToResultVector(ret_buffer, ls.frames, ret_cnt + 16);
				ToResultVector(ret_buffer, ls.ns_total, ret_cnt + 20);
				ToResultVector(ret_buffer, ls.ns_last_frame, ret_cnt + 28);
				ToResultVector(ret_buffer, ls.draws_last_frame, ret_cnt + 36);
				ToResultVector(ret_buffer, ls.qualifying_last_frame, ret_cnt + 40);
				ToResultVector(ret_buffer, ls.hashed_last_frame, ret_cnt + 44);
				ret_cnt += 48;
				buf_cnt += 4;
				break;
			}
			case MsgExtWatchSet:
			{
				// Arguments: u32 start, u32 end (exclusive), u8 cond (1 read, 2 write, 3 both), u32 duration ms (0 = until cleared).
				// Every load/store of the game is instrumented while a watch exists, so the game runs slower: keep windows short.
				if (!VMManager::HasValidVM())
					goto error;
				if (!SafetyChecks(buf_cnt, 4 + 4 + 1 + 4, ret_cnt, 0, buf_size)) [[unlikely]]
					goto error;
				const u32 start = FromSpan<u32>(buf, buf_cnt);
				const u32 end = FromSpan<u32>(buf, buf_cnt + 4);
				const u8 cond = FromSpan<u8>(buf, buf_cnt + 8);
				const u32 duration_ms = FromSpan<u32>(buf, buf_cnt + 9);
				if (start >= end || (cond & 3) == 0)
					goto error;
				Host::RunOnCPUThread([start, end, cond, duration_ms] {
					if (VMManager::HasValidVM())
						IAEmWatch::Set(start, end, cond, duration_ms);
				});
				buf_cnt += 13;
				break;
			}
			case MsgExtWatchRead:
			{
				// Argument: u8 reset (1 = empty the table after copying). Reply: 7 x u32 stats, u32 n, then n x 160-byte entries.
				if (!SafetyChecks(buf_cnt, 1, ret_cnt, 0, buf_size)) [[unlikely]]
					goto error;
				const bool reset = FromSpan<u8>(buf, buf_cnt) != 0;
				std::vector<IAEmWatch::Entry> entries;
				const IAEmWatch::Stats st = IAEmWatch::Read(&entries, reset);
				const u32 n = static_cast<u32>(entries.size());
				if (!SafetyChecks(buf_cnt, 1, ret_cnt, 7 * 4 + 4 + n * sizeof(IAEmWatch::Entry), buf_size)) [[unlikely]]
					goto error;
				ToResultVector(ret_buffer, st.active, ret_cnt);
				ToResultVector(ret_buffer, st.start, ret_cnt + 4);
				ToResultVector(ret_buffer, st.end, ret_cnt + 8);
				ToResultVector(ret_buffer, st.total_hits, ret_cnt + 12);
				ToResultVector(ret_buffer, st.dropped, ret_cnt + 16);
				ToResultVector(ret_buffer, st.entries, ret_cnt + 20);
				ToResultVector(ret_buffer, st.ms_left, ret_cnt + 24);
				ToResultVector(ret_buffer, n, ret_cnt + 28);
				ret_cnt += 32;
				if (n > 0)
				{
					memcpy(&ret_buffer[ret_cnt], entries.data(), n * sizeof(IAEmWatch::Entry));
					ret_cnt += n * sizeof(IAEmWatch::Entry);
				}
				buf_cnt += 1;
				break;
			}
			case MsgExtWatchClear:
			{
				if (!SafetyChecks(buf_cnt, 0, ret_cnt, 0, buf_size)) [[unlikely]]
					goto error;
				Host::RunOnCPUThread([] {
					IAEmWatch::Clear();
				});
				break;
			}
			case MsgExtCall:
			{
				// Arguments: u32 function, u8 argc (0..4), 4 x u32 args, u32 timeout ms. Reply: u8 status (0 done,
				// 1 aborted on timeout, 2 refused: busy or VM not running), u32 v0, u32 v1, u32 elapsed ms.
				// Synchronous: the PINE thread polls the CPU thread at every vsync until the function returns.
				if (!SafetyChecks(buf_cnt, 4 + 1 + 4 * 4 + 4, ret_cnt, 1 + 4 + 4 + 4, buf_size)) [[unlikely]]
					goto error;
				const u32 func = FromSpan<u32>(buf, buf_cnt);
				const u8 argc = FromSpan<u8>(buf, buf_cnt + 4);
				u32 args[IAEmCall::MAX_ARGS];
				for (u32 i = 0; i < IAEmCall::MAX_ARGS; i++)
					args[i] = FromSpan<u32>(buf, buf_cnt + 5 + 4 * i);
				const u32 timeout_ms = FromSpan<u32>(buf, buf_cnt + 21);
				buf_cnt += 25;

				u8 status = 2;
				if (VMManager::HasValidVM() && VMManager::GetState() == VMState::Running && argc <= IAEmCall::MAX_ARGS)
				{
					bool started = false;
					Host::RunOnCPUThread([&started, func, &args, argc] {
						started = VMManager::GetState() == VMState::Running && IAEmCall::Start(func, args, argc);
					}, true);
					if (started)
					{
						const auto deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(timeout_ms);
						bool done = false;
						while (!done)
						{
							Host::RunOnCPUThread([&done] { done = IAEmCall::Poll(); }, true);
							if (done || std::chrono::steady_clock::now() >= deadline || s_end.load(std::memory_order_acquire))
								break;
							std::this_thread::sleep_for(std::chrono::milliseconds(1));
						}
						if (!done)
							Host::RunOnCPUThread([] { IAEmCall::Abort(); }, true);
						status = done ? 0 : 1;
					}
				}
				const IAEmCall::Result r = IAEmCall::Last();
				ToResultVector(ret_buffer, status, ret_cnt);
				ToResultVector(ret_buffer, r.v0, ret_cnt + 1);
				ToResultVector(ret_buffer, r.v1, ret_cnt + 5);
				ToResultVector(ret_buffer, r.elapsed_ms, ret_cnt + 9);
				ret_cnt += 13;
				break;
			}
			default:
			{
			error:
				return IPCBuffer{5, MakeFailIPC(ret_buffer)};
			}
		}
	}
	return IPCBuffer{(int)ret_cnt, MakeOkIPC(ret_buffer, ret_cnt)};
}
