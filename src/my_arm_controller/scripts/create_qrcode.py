import segno

# 生成二维码（H级纠错）
qr = segno.make("ID=001", error="h")  

# 保存为PNG（scale=10 控制像素大小）
# 生成高清二维码（适合打印）
qr.save("qr_highres.png", scale=40, dpi=300)  # dpi控制打印质量 

# 可选：在终端显示二维码（文本模式）
# print(qr.terminal())