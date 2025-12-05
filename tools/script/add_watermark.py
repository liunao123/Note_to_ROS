#!/usr/bin/env python3
"""
给图片添加文件名水印的脚本
"""

import os
import glob
from PIL import Image, ImageDraw, ImageFont
import sys

def add_watermark_to_image(image_path, output_dir="watermarked"):
    """
    给单张图片添加文件名水印
    
    Args:
        image_path: 图片文件路径
        output_dir: 输出目录
    """
    try:
        # 打开图片
        with Image.open(image_path) as img:
            # 检查EXIF方向信息并正确旋转
            try:
                exif = img.getexif()
                orientation = exif.get(274, 1) if exif else 1
                
                # 根据EXIF方向标签正确旋转图片
                if orientation == 3:
                    img = img.rotate(180, expand=True)
                elif orientation == 6:
                    img = img.rotate(270, expand=True)  # 顺时针90度
                elif orientation == 8:
                    img = img.rotate(90, expand=True)   # 逆时针90度
                # orientation == 1 或其他值不需要旋转
                
            except:
                # 如果读取EXIF失败，保持原图
                pass
            
            # 获取正确方向后的图片
            original_img = img.copy()
        
        # 转换为RGBA模式以支持透明度
        if original_img.mode != 'RGBA':
            original_img = original_img.convert('RGBA')
            
            # 创建水印层
            watermark = Image.new('RGBA', original_img.size, (0, 0, 0, 0))
            draw = ImageDraw.Draw(watermark)
            
            # 获取文件名（不包含路径）并去掉后缀，只保留序号
            filename = os.path.basename(image_path)
            # 去掉文件扩展名，只保留序号部分
            display_text = os.path.splitext(filename)[0]
            
            # 计算字体大小（基于图片尺寸，再调大2倍）
            font_size = max(50, min(original_img.width, original_img.height) // 4)
            
            try:
                # 尝试使用系统字体
                font = ImageFont.truetype("/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf", font_size)
            except (OSError, IOError):
                try:
                    # 备用字体
                    font = ImageFont.truetype("/usr/share/fonts/truetype/liberation/LiberationSans-Bold.ttf", font_size)
                except (OSError, IOError):
                    # 如果没有找到字体，使用默认字体
                    font = ImageFont.load_default()
            
            # 获取文本尺寸
            bbox = draw.textbbox((0, 0), display_text, font=font)
            text_width = bbox[2] - bbox[0]
            text_height = bbox[3] - bbox[1]
            
            # 计算水印位置（水平居中，垂直位置在1/4处）
            x = (original_img.width - text_width) // 2  # 水平居中
            y = original_img.height // 4 - text_height // 2  # 垂直1/4处
            
            # 绘制文本阴影（黑色）
            shadow_offset = 5  # 增大阴影偏移以适应更大字体
            draw.text((x + shadow_offset, y + shadow_offset), display_text, 
                     font=font, fill=(0, 0, 0, 180))
            
            # 绘制主文本（红色）
            draw.text((x, y), display_text, font=font, fill=(255, 0, 0, 220))
            
            # 合并水印到原图
            result = Image.alpha_composite(original_img, watermark)
            
            # 转换回RGB模式
            if result.mode == 'RGBA':
                result = result.convert('RGB')
            
            # 统一调整分辨率为300x400
            target_size = (300, 400)
            result_resized = result.resize(target_size, Image.Resampling.LANCZOS)
            
            # 确保输出目录存在
            os.makedirs(output_dir, exist_ok=True)
            
            # 保存结果，明确指定不保存EXIF信息
            output_path = os.path.join(output_dir, filename)
            result_resized.save(output_path, quality=95, exif=b'')
            
            print(f"✓ 已处理: {filename}")
            return True
            
    except Exception as e:
        print(f"✗ 处理失败 {image_path}: {str(e)}")
        return False

def main():
    """主函数"""
    # 支持的图片格式
    image_extensions = ['*.jpg', '*.jpeg', '*.JPG', '*.JPEG', '*.png', '*.PNG']
    
    # 获取当前目录下所有图片文件
    image_files = []
    for ext in image_extensions:
        image_files.extend(glob.glob(ext))
    
    if not image_files:
        print("在当前目录下没有找到图片文件！")
        return
    
    print(f"找到 {len(image_files)} 个图片文件")
    print("开始添加水印...")
    
    success_count = 0
    total_count = len(image_files)
    
    # 处理每个图片文件
    for image_file in sorted(image_files):
        if add_watermark_to_image(image_file):
            success_count += 1
    
    print(f"\n处理完成！")
    print(f"成功处理: {success_count}/{total_count} 个文件")
    print(f"带水印的图片保存在 'watermarked' 目录下")

if __name__ == "__main__":
    main()
