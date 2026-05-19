import PyInstaller.__main__
import shutil
import os
import sys

base_dir = os.path.dirname(os.path.abspath(__file__))
project_root = os.path.dirname(base_dir)

# 定义路径
ui_entry = os.path.join(base_dir, 'UI.py')
src_dir = os.path.join(base_dir, 'src')
sdk_dir = os.path.join(project_root, 'PYTHON_SDK')

if not os.path.exists(sdk_dir):
    print(f"错误：找不到 PYTHON_SDK 目录：{sdk_dir}")
    sys.exit(1)

if sys.platform == 'win32':
    pack_args = [
        ui_entry,
        '--onefile',
        # '--windowed',
        '--icon', os.path.join(src_dir, 'logo.ico'),
        '--name', 'FXPlatform',
        '--add-binary', os.path.join(sdk_dir, 'libGentoSDKPY.dll') + ';.',
        '--paths', sdk_dir,
        '--hidden-import', 'ctypes',
    ]
    # 添加所有 .dll
    pack_args.extend(['--add-binary', os.path.join(sdk_dir, '*.dll') + ';.'])
    # 添加所有 .py 文件到打包中，目标路径为 PYTHON_SDK（运行时相对路径）
    pack_args.extend(['--add-data', os.path.join(sdk_dir, '*.py') + ';PYTHON_SDK'])
    # 添加 src 图标
    pack_args.extend(['--add-data', os.path.join(src_dir, 'logo.ico') + ';src'])
else:
    pack_args = [
        ui_entry,
        '--onefile',
        '--windowed',
        '--icon', os.path.join(src_dir, 'logo.png'),
        '--name', 'FXPlatform',
        '--add-binary', os.path.join(sdk_dir, 'libGentoSDKPY.so') + ':.',
        '--paths', sdk_dir,
        '--hidden-import', 'ctypes',
        '--hidden-import', 'PIL._tkinter_finder',
        '--hidden-import', 'PIL.Image',
        '--hidden-import', 'PIL.ImageTk',
    ]
    pack_args.extend(['--add-binary', os.path.join(sdk_dir, '*.so') + ':.'])
    pack_args.extend(['--add-data', os.path.join(sdk_dir, '*.py') + ':PYTHON_SDK'])
    pack_args.extend(['--add-data', os.path.join(src_dir, 'logo.png') + ':src'])



PyInstaller.__main__.run(pack_args)

print("清理临时文件...")
shutil.rmtree(os.path.join(base_dir, 'build'), ignore_errors=True)
shutil.rmtree(os.path.join(base_dir, '__pycache__'), ignore_errors=True)
# 清理 PYTHON_SDK 下的 pycache（可选）
sdk_pycache = os.path.join(sdk_dir, '__pycache__')
shutil.rmtree(sdk_pycache, ignore_errors=True)

print("打包完成！可执行文件在 dist 文件夹中")