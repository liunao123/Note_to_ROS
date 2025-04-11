#!/usr/bin/python
# -*- coding: utf-8 -*-


from PIL import Image

# 打开图像文件
image = Image.open("/home/11.png")

# 保存图像时指定最高质量
image.save("/home/opt_o.jpg", quality=100)