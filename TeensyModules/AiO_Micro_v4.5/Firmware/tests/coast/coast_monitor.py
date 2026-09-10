"""Coast monitor: captures the AiO Micro v4.5 coast diagnostics over Ethernet and sends commands.
No USB cable needed. Run on any laptop on the tractor network (same subnet as the AiO, e.g. 192.168.5.x).

  python coast_monitor.py                         # listen on UDP 5125, print + append to coast_<date>.log
  python coast_monitor.py --log mylog.txt
  python coast_monitor.py --module 192.168.5.126 --force 10           # forced 10 s coast (GNSS stays good)
  python coast_monitor.py --module 192.168.5.126 --geometry 2.8 0.1 3.0 -0.35   # L pivot height offset (m, AOG signs)

Lines received: $COASTMSG (build/geometry/events), $COASTSHADOW (one per shadow window), $COASTREPORT (live
coast end), $COAST (10 Hz raw log when COAST_LOG is on; feed the saved file to coast_ref.py after converting).
The module's UDP command port is 8888 (the autosteer port); its IP is the one AgIO shows for the steer module.
"""
import argparse, datetime, socket, sys


def send(module_ip, text):
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    s.sendto((text + "\n").encode(), (module_ip, 8888))
    s.close()
    print(f"sent to {module_ip}:8888 -> {text}")


def listen(port, log_path):
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    s.bind(("", port))
    print(f"listening on UDP {port}, logging to {log_path} (Ctrl-C to stop)")
    with open(log_path, "a", encoding="utf-8") as f:
        while True:
            data, addr = s.recvfrom(2048)
            for line in data.decode(errors="replace").splitlines():
                line = line.strip()
                if not line:
                    continue
                stamp = datetime.datetime.now().strftime("%H:%M:%S.%f")[:-3]
                out = f"{stamp} {addr[0]} {line}"
                print(out)
                f.write(out + "\n")
                f.flush()


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", type=int, default=5125)
    ap.add_argument("--log", default=None)
    ap.add_argument("--module", default=None, help="steer module IP for commands")
    ap.add_argument("--force", type=int, default=None, help="forced coast seconds")
    ap.add_argument("--geometry", nargs=4, type=float, metavar=("L", "PIVOT", "HEIGHT", "OFFSET"))
    a = ap.parse_args()

    if a.force is not None or a.geometry:
        if not a.module:
            sys.exit("--module <ip> is required for commands")
        if a.force is not None:
            send(a.module, f"!AOGCO,{a.force}")
        if a.geometry:
            send(a.module, "!AOGCG," + ",".join(f"{v:.3f}" for v in a.geometry))
        if a.log is None:
            sys.exit(0)

    try:
        listen(a.port, a.log or datetime.datetime.now().strftime("coast_%Y%m%d_%H%M%S.log"))
    except KeyboardInterrupt:
        pass
