import os

# 获取当前脚本所在的目录，这将是项目的根目录
project_root = os.path.dirname(os.path.realpath(__file__))

clangd_content = """CompileFlags:
  Add: ["""

# 定义路径列表，每个路径都是相对于项目根目录的绝对路径
paths = [
    os.path.join(project_root, "Core", "Inc"),
    os.path.join(project_root, "Drivers", "CMSIS", "Device", "ST", "STM32F4xx", "Include"),
    os.path.join(project_root, "Drivers", "CMSIS", "Device", "Include"),
    os.path.join(project_root, "Drivers", "STM32F4xx_HAL_Driver", "Inc"),
    os.path.join(project_root, "Drivers", "CMSIS", "Include"),
    os.path.join(project_root, "RC9CPP_API", "CONTROL", "STRUCTURE"),
    os.path.join(project_root, "Middlewares", "Third_Party", "FreeRTOS", "Source", "CMSIS_RTOS_V2"),
    os.path.join(project_root, "Middlewares", "Third_Party", "FreeRTOS", "Source", "include"),
    os.path.join(project_root, "Middlewares", "ST", "ARM", "DSP", "Inc"),
    os.path.join(project_root, "RC9CPP_API", "CONTROL"),
    os.path.join(project_root, "RC9CPP_API", "DECISION"),
    os.path.join(project_root, "RC9CPP_API", "HARDWARE"),
    os.path.join(project_root, "RC9CPP_API", "HARDWARE", "MOTOR"),
    os.path.join(project_root, "RC9CPP_API", "MATH_LIB"),
    os.path.join(project_root, "RC9CPP_API"),
    os.path.join(project_root, "RC9CPP_USER", "R1N_USER"),
    os.path.join(project_root, "RC9CPP_USER", "R2N_USER"),
    os.path.join(project_root, "RC9CPP_USER", "structure_test")
]

# 将路径转换为 -I<absolute_path> 格式，并加入到 clangd_content 字符串中
clangd_content += ",\n".join(f"-I{path}" for path in paths) + "\n  ]"

# 写入 .clangd 文件
with open(".clangd", "w") as f:
    f.write(clangd_content)