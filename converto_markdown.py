from selenium import webdriver
from selenium.webdriver.chrome.options import Options
import html2text
import requests
import base64
import os
from bs4 import BeautifulSoup

# 配置 Chrome 浏览器（无头模式）
chrome_options = Options()
chrome_options.add_argument("--headless")
chrome_options.add_argument("--no-sandbox")
driver = webdriver.Chrome(options=chrome_options)

try:
    # 1. 获取网页内容
    url = "http://wiki.mintasca.com/wiki/cn/#!pages/Ethernet_Communication_Protocol.md"
    driver.get(url)
    html_content = driver.page_source
    driver.quit()  # 关闭浏览器

    # 2. 解析 HTML，提取图片 URL
    soup = BeautifulSoup(html_content, 'html.parser')
    img_tags = soup.find_all('img')
    img_urls = [img['src'] for img in img_tags if 'src' in img.attrs]

    # 3. 转换为 Markdown（初步转换）
    h = html2text.HTML2Text()
    h.ignore_images = False  # 允许图片标记
    markdown = h.handle(str(soup))

    # 4. 下载图片并转换为 Base64
    for i, img_url in enumerate(img_urls):
        try:
            # 下载图片数据
            img_data = requests.get(img_url, stream=True, timeout=10).content
            img_base64 = base64.b64encode(img_data).decode('utf-8')

            # 替换 Markdown 中的图片 URL 为 Base64 数据
            markdown = markdown.replace(
                f"![Image_{i+1}]({img_url})",
                f"![Image_{i+1}](data:image/png;base64,{img_base64})"
            )
        except Exception as e:
            print(f"⚠️ 图片处理失败: {img_url}, 错误: {str(e)}")

    # 5. 保存 Markdown 文件（包含 Base64 图片）
    output_path = "output_with_base64_images.md"
    with open(output_path, "w", encoding="utf-8") as f:
        f.write(markdown)

    print(f"✅ Markdown 文件已保存到: {os.path.abspath(output_path)}")

except Exception as e:
    print(f"❌ 发生错误: {str(e)}")
    if 'driver' in locals():
        driver.quit()