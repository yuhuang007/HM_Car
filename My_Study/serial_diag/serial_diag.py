# Hi3861 串口诊断工具 (纯标准库版本，无需安装任何依赖)
# 用法: python serial_diag.py
import sys, os, time, ctypes, ctypes.wintypes, glob, struct

kernel32 = ctypes.windll.kernel32

def list_com_ports():
    ports = []
    try:
        import winreg
        key = winreg.OpenKey(winreg.HKEY_LOCAL_MACHINE, r"HARDWARE\DEVICEMAP\SERIALCOMM")
        i = 0
        while True:
            try:
                _, val_data, _ = winreg.EnumValue(key, i)
                ports.append(val_data); i += 1
            except WindowsError:
                break
        winreg.CloseKey(key)
    except:
        pass
    for p in glob.glob("COM*"):
        if p not in ports: ports.append(p)
    return sorted(set(ports))

def open_serial(port, baud=115200):
    handle = kernel32.CreateFileW(f"\\\\.\\{port}", 0xC0000000, 0, None, 3, 0x80, None)
    if handle == -1:
        return None
    # DCB: 28 bytes
    dcb = bytearray(28)
    struct.pack_into('<I', dcb, 0, 28)
    struct.pack_into('<I', dcb, 4, baud)
    struct.pack_into('<I', dcb, 8, 0x00000001)  # fBinary=1
    struct.pack_into('B', dcb, 20, 8)  # ByteSize
    struct.pack_into('B', dcb, 21, 0)  # Parity=None
    struct.pack_into('B', dcb, 22, 0)  # StopBits=0
    kernel32.SetCommState(handle, ctypes.byref((ctypes.c_byte * 28)(*dcb)))
    # COMMTIMEOUTS: 20 bytes
    ct = bytearray(20)
    struct.pack_into('<I', ct, 0, 0xFFFFFFFF)  # ReadIntervalTimeout
    struct.pack_into('<I', ct, 4, 0)            # ReadTotalTimeoutMultiplier
    struct.pack_into('<I', ct, 8, 100)          # ReadTotalTimeoutConstant
    struct.pack_into('<I', ct, 12, 0)           # WriteTotalTimeoutMultiplier
    struct.pack_into('<I', ct, 16, 100)         # WriteTotalTimeoutConstant
    kernel32.SetCommTimeouts(handle, ctypes.byref((ctypes.c_byte * 20)(*ct)))
    return handle

def read_serial(handle, size=512):
    buf = (ctypes.c_byte * size)()
    n = ctypes.c_ulong(0)
    if kernel32.ReadFile(handle, buf, size, ctypes.byref(n), None) and n.value > 0:
        return bytes(buf[:n.value])
    return b""

def close_serial(handle):
    kernel32.CloseHandle(handle)

def parse_json_fields(s):
    result = {}
    for key in ['distance', 'carPower', 'L_speed', 'R_speed', 'lineOut', 'lineType']:
        tag = f'"{key}":'
        if tag in s:
            idx = s.index(tag) + len(tag)
            val = ""
            while idx < len(s) and s[idx].isdigit():
                val += s[idx]; idx += 1
            if val: result[key] = int(val)
    return result

def find_binary_frames(data):
    results = []
    i = 0
    while i < len(data) - 3:
        if data[i] == 0xFF:
            h, l, s = data[i+1], data[i+2], data[i+3]
            if ((0xFF + h + l) & 0xFF) == s:
                results.append(('DIST', (h << 8) | l)); i += 4; continue
        elif data[i] == 0xAA:
            lo, lt, s = data[i+1], data[i+2], data[i+3]
            if ((0xAA + lo + lt) & 0xFF) == s:
                results.append(('LINE', lo, lt)); i += 4; continue
        i += 1
    return results

def main():
    print("=" * 60)
    print("  Hi3861 串口诊断工具 (无需安装依赖)")
    print("  用途: 检测超声波/循迹模块是否在发送数据")
    print("=" * 60)

    ports = list_com_ports()
    if not ports:
        print("\n  未检测到串口! 请检查 USB 连接和驱动")
        input("  按回车退出..."); return
    print("\n  可用串口:")
    for i, p in enumerate(ports): print(f"    [{i+1}] {p}")

    if len(ports) == 1:
        port = ports[0]; print(f"  自动选择: {port}")
    else:
        choice = input("  选择串口号 (输入数字): ").strip()
        try: port = ports[int(choice) - 1]
        except: port = ports[0]; print(f"  使用默认: {port}")

    baud = 115200
    print(f"\n  连接: {port} @ {baud}")
    print("  按 Ctrl+C 停止\n")

    handle = open_serial(port, baud)
    if not handle:
        print(f"  打开 {port} 失败! 可能被其他程序占用。")
        input("  按回车退出..."); return

    json_count = dist_bin = line_bin = 0
    dist_vals = set(); line_out_vals = set(); line_type_vals = set()
    t0 = time.time(); last_report = t0
    buf = b""; raw_buf = b""

    try:
        while True:
            data = read_serial(handle, 512)
            if not data: time.sleep(0.01); continue

            raw_buf += data
            buf += data

            while b'{' in buf and b'}' in buf:
                try:
                    si = buf.index(b'{'); ei = buf.index(b'}', si) + 1
                    fields = parse_json_fields(buf[si:ei].decode('utf-8', errors='ignore'))
                    buf = buf[ei:]
                    if fields:
                        json_count += 1
                        if 'distance' in fields: dist_vals.add(fields['distance'])
                        if 'lineOut' in fields: line_out_vals.add(fields['lineOut'])
                        if 'lineType' in fields: line_type_vals.add(fields['lineType'])
                except: break

            frames = find_binary_frames(raw_buf)
            raw_buf = raw_buf[-10:]
            for f in frames:
                if f[0] == 'DIST': dist_bin += 1; dist_vals.add(f[1])
                elif f[0] == 'LINE': line_bin += 1; line_out_vals.add(f[1]); line_type_vals.add(f[2])

            now = time.time()
            if now - last_report >= 3:
                print(f"--- 运行 {now-t0:.0f}s ---")
                print(f"  JSON帧数:          {json_count}")
                print(f"  距离二进制帧(0xFF): {dist_bin}")
                print(f"  循迹二进制帧(0xAA): {line_bin}")
                print(f"  distance值:         {sorted(dist_vals) if dist_vals else '无'}")
                print(f"  lineOut值:          {sorted(line_out_vals) if line_out_vals else '无'}")
                print(f"  lineType值:         {sorted(line_type_vals) if line_type_vals else '无'}")
                print()
                if json_count > 0:
                    if dist_vals == {0}:
                        print("  [!!] 距离始终=0 -> CS100A超声波模块未工作!")
                    elif dist_vals: print("  [OK] 超声波有数据")
                    if not line_out_vals and not line_bin:
                        print("  [!!] 无循迹数据 -> 光电管模块未工作!")
                    elif line_out_vals:
                        lo = max(line_out_vals)
                        print(f"  [OK] 循迹数据正常, 光电管: {' '.join([f'S{i}:{(lo>>i)&1}' for i in range(8)])}")
                print()
                last_report = now
    except KeyboardInterrupt:
        print("\n  停止监听")
    finally:
        close_serial(handle)

if __name__ == "__main__":
    main()
