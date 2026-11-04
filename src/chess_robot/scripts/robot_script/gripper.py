import argparse
from onrobot_rg_control.remote_gripper import RemoteGripper, init, shutdown

def set_gripper(width: int, force: int = 200) -> None:
    init()
    gripper = RemoteGripper(gripper='rg2ft')

    try:
        if not gripper.wait_for_connection():
            raise RuntimeError('The gripper TCP node is not running')

        gripper.set(width=width, force=force)

        status = gripper.get_status()
        if status is None:
            raise RuntimeError('No gripper status received')

        print(f'width={status.grip_width}, busy={status.busy}')
    finally:
        gripper.destroy_node()
        shutdown()

if __name__ == '__main__':

    parser = argparse.ArgumentParser()
    parser.add_argument('--width', type=int, required=True)
    parser.add_argument('--force', type=int, default=200)
    args = parser.parse_args()

    set_gripper(width=args.width*10, force=args.force*10)