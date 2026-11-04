from pymodbus.client import ModbusTcpClient
import time

def read_gripper_status(ip: str):
    # Connect to the OnRobot Compute Box Modbus server
    client = ModbusTcpClient(ip, port=502)
    if not client.connect():
        print("Failed to connect to Compute Box.")
        return

    UNIT_ID = 65  # Slave ID for Port 1

    try:
        # Read Input Registers starting at address 258 (Function Code 4)
        # We read 10 registers from 258 to capture width, force, and status flags
        response = client.read_input_registers(address=258, count=10, device_id=UNIT_ID)

        if response.isError():
            print(f"Modbus Read Error: {response}")
        else:
            # Register 258 (index 0 in our read block) is Current Width
            raw_width = response.registers[0]
            current_width_mm = raw_width / 10.0

            # Register 267 (index 9 in our read block) is Status / Error Code
            status_code = response.registers[9]

            print(f"--- OnRobot Status Report ---")
            print(f"Current Finger Width: {current_width_mm:.1f} mm")
            print(f"System Status / Error Code: {status_code}")
            
            if status_code != 0:
                print(f"WARNING: Gripper reports a non-zero status code (Error/Busy flag active).")
            else:
                print("Gripper status is nominal (No errors reported).")

    except Exception as e:
        print(f"Exception during read: {e}")
    finally:
        client.close()

if __name__ == "__main__":
    COMPUTE_BOX_IP = '10.150.0.2'
    read_gripper_status(COMPUTE_BOX_IP)