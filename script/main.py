import sys
import time
import select

# 导入核心控制包 (假设 sf_can_controller.py 位于同一目录下)
from sf_can_controller import MotorController 

# --- 核心配置 ---
IFACE = "can0"        
MOTOR_ID = 1          
UPDATE_RATE_HZ = 100.0 
PRINT_EVERY = 2     
INITIAL_TARGET_DEG = 0.0

# --- 主控制循环 ---
def run_simple_test() -> None:
    """运行简化的位置控制循环。"""
    
    # 1. 初始化
    update_period = 1.0 / UPDATE_RATE_HZ
    target_rad = INITIAL_TARGET_DEG
    
    KP, KD = 0.5, 0.3 # 默认 MIT 参数
    
    controller = MotorController(interface=IFACE, motor_id=MOTOR_ID)
    print(f"--- SF Motor Test Start ---")
    print(f"接口: {IFACE}, ID: {MOTOR_ID}, 频率: {UPDATE_RATE_HZ} Hz")
    
    # 2. 使能
    controller.enable()
    
    last_send_time = time.perf_counter()
    print_counter = 0

    inputCheckCount = 0;
    # 3. 主循环
    while True:
        controller.poll_rx()
        current_state = controller.get_motor_state()
        
        now = time.perf_counter()
        
        # --- 周期性输入检查 (每 100 个循环) ---
        inputCheckCount += 1
        if inputCheckCount >= 500:
            inputCheckCount = 0
            
            # 使用阻塞 I/O 等待用户输入（会暂停控制循环）
            # 注意：如果输入非数字，程序将抛出 ValueError 崩溃。
            line = input("请输入目标关节角度: ").strip()
            if line:
                angle_deg = float(line)
                target_rad = angle_deg
                print(f"已更新目标关节角度：{angle_deg:.3f} °")
        
        # 周期性发送 MIT 命令
        if now - last_send_time >= update_period:
            last_send_time = now
            
            # 发送目标位置命令
            controller.send_mit_command(
                pos=target_rad,
                vel=0.0,
                kp=KP,
                kd=KD,
                tor=0.0
            )

            # 打印状态
            print_counter += 1
            if print_counter >= PRINT_EVERY:
                print_counter = 0
                print(
                    f"Cmd={target_rad:.2f} | "
                    f"Pos={current_state.pos:.2f} (Vel={current_state.vel:.2f})"
                )
        
        time.sleep(0.001)
            

if __name__ == "__main__":
    # 运行测试
    run_simple_test()