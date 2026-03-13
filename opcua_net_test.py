#!/usr/bin/env python3
import socket
import time
from opcua import Client

ENDPOINTS = [
    ("Leitstand",     "opc.tcp://192.168.97.101:4840"),
    ("Palettenlager", "opc.tcp://192.168.97.102:4840"),
    ("Handling",      "opc.tcp://192.168.97.103:4840"),
    ("Presse_Press",  "opc.tcp://192.168.97.104:4840"),
]

USER = "MES"
PASSWORD = "training"
TCP_TIMEOUT_S = 2.0
OPCUA_TIMEOUT_S = 5.0


def tcp_probe(host: str, port: int) -> (bool, str):
    """Testa se dá pra abrir conexão TCP host:port."""
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    s.settimeout(TCP_TIMEOUT_S)
    t0 = time.time()
    try:
        s.connect((host, port))
        dt = (time.time() - t0) * 1000.0
        return True, f"TCP OK ({dt:.0f} ms)"
    except Exception as e:
        return False, f"TCP FAIL: {e}"
    finally:
        try:
            s.close()
        except Exception:
            pass


def opcua_connect_test(url: str) -> (bool, str):
    """Testa handshake OPC UA com user/senha."""
    c = Client(url, timeout=OPCUA_TIMEOUT_S)
    c.set_user(USER)
    c.set_password(PASSWORD)
    try:
        c.connect()
        # leitura mínima só pra confirmar sessão
        _ = c.get_root_node().get_browse_name()
        return True, "OPC UA CONNECT OK"
    except Exception as e:
        return False, f"OPC UA FAIL: {e}"
    finally:
        try:
            c.disconnect()
        except Exception:
            pass


def parse_host_port(opc_url: str):
    # "opc.tcp://IP:PORT"
    s = opc_url.replace("opc.tcp://", "")
    host, port = s.split(":")
    return host, int(port)


def main():
    print("=== OPC UA Network Test (python-opcua) ===")
    print(f"User={USER}  TimeoutTCP={TCP_TIMEOUT_S}s  TimeoutOPCUA={OPCUA_TIMEOUT_S}s\n")

    for name, url in ENDPOINTS:
        host, port = parse_host_port(url)
        print(f"[{name}] {url}")

        ok_tcp, msg_tcp = tcp_probe(host, port)
        print("  -", msg_tcp)

        if ok_tcp:
            ok_ua, msg_ua = opcua_connect_test(url)
            print("  -", msg_ua)
        else:
            print("  - Skipping OPC UA (no TCP)")

        print()

    print("Done.")


if __name__ == "__main__":
    main()
