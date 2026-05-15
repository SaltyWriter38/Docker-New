# notam_smoke_test.py
import socket, time

s = socket.socket()
s.connect(("127.0.0.1", 9091))

def send(line: str):
    print(f">>> {line.strip()}")
    s.sendall(line.encode("utf-8"))
    time.sleep(0.3)  # let logs flush before next send

# --- Should publish ---
send('<notam>{"x": 9.1, "y": -95.5, "radius": 7.7}\n')
send('<notam>{"x": 25.0, "y": 30.0, "radius": 12.0}\n')
send('<notam>{"id": "notam-0", "x": 10.0, "y": -90.0, "radius": 8.0}\n')   # update
send('<notam>{"id": "low-zone", "x": 5.0, "y": 5.0, "radius": 3.0, "alt_min": 0.0, "alt_max": 50.0}\n')
send('<notam>[15.0, 15.0, 4.5]\n')                                         # raw list

# --- Should be REJECTED with warnings ---
send('<notam>{"x": 1, "y": 2}\n')                  # missing radius
send('<notam>{"x": 1, "y": 2, "radius": -3}\n')    # negative radius
send('<notam>{"x": "nope", "y": 2, "radius": 5}\n')# non-numeric
send('<notam>not even json\n')                     # garbage

s.close()
print("Done.")
