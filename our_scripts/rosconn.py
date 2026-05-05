#!/usr/bin/env python3
"""
Socket Messenger - Read/Write on a localhost port with message routing.
Message format: <route>message
Example:        <chat>Hello world!
"""

import socket
import threading
import sys
import argparse
from datetime import datetime


def parse_message(raw: str) -> tuple[str, str]:
    """Parse '<route>message' format. Returns (route, message)."""
    raw = raw.strip()
    if raw.startswith("<"):
        end = raw.find(">")
        if end != -1:
            route = raw[1:end]
            message = raw[end + 1:]
            return route, message
    return "default", raw


def timestamp() -> str:
    return datetime.now().strftime("%H:%M:%S")


# ─────────────────────────────────────────────
#  SERVER MODE  (listens for incoming connections)
# ─────────────────────────────────────────────

def handle_client(conn: socket.socket, addr: tuple, routes: dict):
    """Receive messages from a connected client and dispatch by route."""
    print(f"[{timestamp()}] Client connected: {addr[0]}:{addr[1]}")
    buffer = ""
    try:
        while True:
            data = conn.recv(4096)
            if not data:
                break
            buffer += data.decode("utf-8")
            # Process all complete newline-terminated messages
            while "\n" in buffer:
                line, buffer = buffer.split("\n", 1)
                if not line.strip():
                    continue
                route, message = parse_message(line)
                handler = routes.get(route, routes.get("default"))
                if handler:
                    handler(route, message, conn, addr)
                else:
                    print(f"[{timestamp()}] [{route}] {message}")
    except (ConnectionResetError, BrokenPipeError):
        pass
    finally:
        print(f"[{timestamp()}] Client disconnected: {addr[0]}:{addr[1]}")
        conn.close()


def writer_thread(clients: list, stop_event: threading.Event):
    """Read from stdin and broadcast to all connected clients."""
    print("Type  <route>message  to send (e.g. <chat>Hello). Ctrl-C to quit.\n")
    while not stop_event.is_set():
        try:
            line = input()
        except EOFError:
            break
        if not line.strip():
            continue
        packet = (line.strip() + "\n").encode("utf-8")
        dead = []
        for c in list(clients):
            try:
                c.sendall(packet)
            except OSError:
                dead.append(c)
        for c in dead:
            clients.remove(c)


def run_server(host: str, port: int, routes: dict):
    clients: list[socket.socket] = []
    stop_event = threading.Event()

    server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    server.bind((host, port))
    server.listen(5)
    print(f"[{timestamp()}] Server listening on {host}:{port}")

    # Writer runs in background so we can also type messages
    t = threading.Thread(target=writer_thread, args=(clients, stop_event), daemon=True)
    t.start()

    try:
        while True:
            conn, addr = server.accept()
            clients.append(conn)
            threading.Thread(
                target=handle_client,
                args=(conn, addr, routes),
                daemon=True,
            ).start()
    except KeyboardInterrupt:
        print("\n[server] Shutting down.")
    finally:
        stop_event.set()
        server.close()


# ─────────────────────────────────────────────
#  CLIENT MODE  (connects to a running server)
# ─────────────────────────────────────────────

def reader_thread(sock: socket.socket, routes: dict, stop_event: threading.Event):
    """Continuously receive messages from the server."""
    buffer = ""
    try:
        while not stop_event.is_set():
            data = sock.recv(4096)
            if not data:
                print(f"\n[{timestamp()}] Server closed the connection.")
                stop_event.set()
                break
            buffer += data.decode("utf-8")
            while "\n" in buffer:
                line, buffer = buffer.split("\n", 1)
                if not line.strip():
                    continue
                route, message = parse_message(line)
                handler = routes.get(route, routes.get("default"))
                if handler:
                    handler(route, message, sock, None)
                else:
                    print(f"[{timestamp()}] [{route}] {message}")
    except OSError:
        pass


def run_client(host: str, port: int, routes: dict):
    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    try:
        sock.connect((host, port))
    except ConnectionRefusedError:
        print(f"[error] Could not connect to {host}:{port} — is the server running?")
        sys.exit(1)

    print(f"[{timestamp()}] Connected to {host}:{port}")
    print("Type  <route>message  to send (e.g. <chat>Hello). Ctrl-C to quit.\n")

    stop_event = threading.Event()
    t = threading.Thread(
        target=reader_thread, args=(sock, routes, stop_event), daemon=True
    )
    t.start()

    try:
        while not stop_event.is_set():
            try:
                line = input()
            except EOFError:
                break
            if not line.strip():
                continue
            try:
                sock.sendall((line.strip() + "\n").encode("utf-8"))
            except OSError:
                print("[error] Lost connection to server.")
                break
    except KeyboardInterrupt:
        print("\n[client] Disconnecting.")
    finally:
        stop_event.set()
        sock.close()


# ─────────────────────────────────────────────
#  ROUTE HANDLERS  — customise these freely
# ─────────────────────────────────────────────

def on_chat(route, message, conn, addr):
    src = f"{addr[0]}:{addr[1]}" if addr else "server"
    print(f"[{timestamp()}] [{route}] ({src}) {message}")


def on_cmd(route, message, conn, addr):
    print(f"[{timestamp()}] [COMMAND] {message}")
    # Example: echo an ACK back to sender
    if conn:
        ack = f"<ack>Command received: {message}\n"
        try:
            conn.sendall(ack.encode("utf-8"))
        except OSError:
            pass


def on_default(route, message, conn, addr):
    print(f"[{timestamp()}] default [{route}] {message}")

def on_destination(route, message, conn, addr):
    print(f"[{timestamp()}]  [{route}] changing flight route to {message}")

def on_data(route, message, conn, addr):
    print(f"[{timestamp()}] [{route}] {message}")

def on_docked(route, message, conn, addr):
    print(f"[{timestamp()}] [{route}] {message}") 


# Map route names -> handler functions
ROUTES: dict = {
    "chat":    on_chat,
    "cmd":     on_cmd,
    "default": on_default,
    "destination": on_destination,
    "delivered": on_data,
    "data": on_data,
    "docked": on_docked,
}


# ─────────────────────────────────────────────
#  ENTRY POINT
# ─────────────────────────────────────────────

def main():
    parser = argparse.ArgumentParser(
        description="Socket messenger with <route>message protocol."
    )
    parser.add_argument(
        "mode", choices=["server", "client"],
        help="Run as server (listen) or client (connect).",
    )
    parser.add_argument("--host", default="127.0.0.1", help="Host (default: 127.0.0.1)")
    parser.add_argument("--port", type=int, default=9091, help="Port (default: 9091)")
    args = parser.parse_args()

    if args.mode == "server":
        run_server(args.host, args.port, ROUTES)
    else:
        run_client(args.host, args.port, ROUTES)


if __name__ == "__main__":
    main()