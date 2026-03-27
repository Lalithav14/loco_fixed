"""
extract_frame_hex_5mp.py
Extracts a 5MP (2592x1944) YUV 4:2:0 frame into per-block hex files.
Usage: python3 extract_frame_hex_5mp.py your_5mp_frame.yuv

Outputs:
  frame_y_5mp.hex  — all Y blocks concatenated (for $readmemh)
  frame_u_5mp.hex  — all U blocks
  frame_v_5mp.hex  — all V blocks

Block counts (with ceiling padding):
  Y : ceil(1944/32) x ceil(2592/32) = 61 x 81 = 4941 blocks
  UV: ceil( 972/16) x ceil(1296/16) = 61 x 81 = 4941 blocks each
"""
import sys, math, os

YUV  = sys.argv[1] if len(sys.argv)>1 else "frame_5mp.yuv"
W, H = 2592, 1944

if not os.path.exists(YUV):
    print(f"ERROR: {YUV} not found.")
    print(f"Usage: python3 extract_frame_hex_5mp.py your_5mp_file.yuv")
    sys.exit(1)

raw     = open(YUV,'rb').read()
Y_size  = W * H
UV_size = (W//2) * (H//2)
print(f"File size: {len(raw)} bytes, expected: {Y_size + 2*UV_size}")

planes = {
    'y': (0,           W,    H,    32, 32),
    'u': (Y_size,      W//2, H//2, 16, 16),
    'v': (Y_size+UV_size, W//2, H//2, 16, 16),
}

for plane,(offset,fw,fh,mbw,mbh) in planes.items():
    cols = math.ceil(fw/mbw)
    rows = math.ceil(fh/mbh)
    all_px = []
    blk = 0
    for r in range(rows):
        for c in range(cols):
            for py in range(mbh):
                for px in range(mbw):
                    fy = min(r*mbh+py, fh-1)
                    fx = min(c*mbw+px, fw-1)
                    idx = offset + fy*fw + fx
                    val = raw[idx] if idx < len(raw) else 128
                    all_px.append(val)
            blk += 1
    fname = f"frame_{plane}_5mp.hex"
    with open(fname,'w') as f:
        for v in all_px:
            f.write(f"{v:02x}\n")
    print(f"{plane.upper()}: {blk} blocks -> {fname}  ({len(all_px)} bytes)")
    if fh%mbh!=0:
        print(f"      {mbh-(fh%mbh)} padding rows added (edge-extended)")

print("\nDone. Use tb_quick_5mp.v to test a sample, tb_full_5mp.v for the full frame.")
print("Block counts: Y=4941, UV=4941 each")
