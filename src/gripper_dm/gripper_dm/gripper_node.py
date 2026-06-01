import os
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
from sensor_msgs.msg import JointState
from builtin_interfaces.msg import Time
from std_srvs.srv import Trigger

from gripper_dm_msgs.srv import SetGripper
from gripper_dm.canfd_driver import CANFDDriver
from gripper_dm.motor_protocol import MotorProtocol
from gripper_dm.gripper_controller import DMGripperController, GripperState


class GripperNode(Node):
    def __init__(self):
        super().__init__('gripper_node')

        self.declare_parameter('lib_path', '')
        self.declare_parameter('can_id', 0x02)
        self.declare_parameter('mst_id', 0x12)
        self.declare_parameter('channel_index', 0)
        self.declare_parameter('abit_baud', 1000000)
        self.declare_parameter('dbit_baud', 5000000)
        self.declare_parameter('open_position', 0.1)
        self.declare_parameter('close_position', 1.1)
        self.declare_parameter('max_speed', 1.0)
        self.declare_parameter('kp_move', 10.0)
        self.declare_parameter('kd_move', 0.5)
        self.declare_parameter('hold_torque', 1.0)
        self.declare_parameter('position_tolerance', 0.02)
        self.declare_parameter('stall_speed_threshold', 0.1)
        self.declare_parameter('stall_torque_threshold', 0.3)
        self.declare_parameter('control_rate', 100.0)
        self.declare_parameter('joint_name', 'gripper_joint')

        lib_path = self._resolve_lib_path()
        self._channel_index = self.get_parameter('channel_index').value
        self._abit_baud = self.get_parameter('abit_baud').value
        self._dbit_baud = self.get_parameter('dbit_baud').value
        self._joint_name = self.get_parameter('joint_name').value

        params = {
            'open_position': self.get_parameter('open_position').value,
            'close_position': self.get_parameter('close_position').value,
            'max_speed': self.get_parameter('max_speed').value,
            'kp_move': self.get_parameter('kp_move').value,
            'kd_move': self.get_parameter('kd_move').value,
            'hold_torque': self.get_parameter('hold_torque').value,
            'position_tolerance': self.get_parameter('position_tolerance').value,
            'stall_speed_threshold': self.get_parameter('stall_speed_threshold').value,
            'stall_torque_threshold': self.get_parameter('stall_torque_threshold').value,
            'control_rate': self.get_parameter('control_rate').value,
        }

        self._driver = CANFDDriver(lib_path)
        self._motor = MotorProtocol(
            self._driver,
            can_id=self.get_parameter('can_id').value,
            mst_id=self.get_parameter('mst_id').value,
        )
        self._controller = DMGripperController(self._motor, params)

        self._connect()

        cb_group = ReentrantCallbackGroup()

        self._srv_set = self.create_service(
            SetGripper, 'set_gripper', self._on_set_gripper,
            callback_group=cb_group
        )
        self._srv_open = self.create_service(
            Trigger, 'open_gripper', self._on_open_gripper,
            callback_group=cb_group
        )
        self._srv_close = self.create_service(
            Trigger, 'close_gripper', self._on_close_gripper,
            callback_group=cb_group
        )
        self._srv_reconnect = self.create_service(
            Trigger, 'reconnect_gripper', self._on_reconnect,
            callback_group=cb_group
        )

        rate = self.get_parameter('control_rate').value
        self._timer = self.create_timer(1.0 / rate, self._control_loop)
        self._pub_joint_state = self.create_publisher(JointState, 'joint_states', 10)

        self.get_logger().info(f'GripperNode started at {rate} Hz')

    def _resolve_lib_path(self):
        lib_path = self.get_parameter('lib_path').value
        if lib_path and os.path.isfile(lib_path):
            return lib_path
        pkg_dir = os.path.dirname(os.path.abspath(__file__))
        path = os.path.join(pkg_dir, 'libcontrolcanfd.so')
        if os.path.isfile(path):
            return path
        from ament_index_python.packages import get_package_share_directory
        try:
            pkg_share = get_package_share_directory('gripper_dm')
            path = os.path.join(pkg_share, 'libcontrolcanfd.so')
            if os.path.isfile(path):
                return path
        except Exception:
            pass
        raise FileNotFoundError("libcontrolcanfd.so not found")

    def _connect(self):
        try:
            self._driver.open_device()
            self._driver.init_channel(
                channel_index=self._channel_index,
                abit_baud=self._abit_baud,
                dbit_baud=self._dbit_baud,
            )
            self._controller.initialize()
            self.get_logger().info('Device connected and motor enabled')
        except Exception as e:
            self.get_logger().error(f'Failed to connect: {e}')

    def _on_set_gripper(self, request, response):
        try:
            if request.close:
                self._controller.close_gripper(request.hold_torque)
                response.success = True
                response.message = 'Closing gripper'
            else:
                self._controller.open_gripper()
                response.success = True
                response.message = 'Opening gripper'
            response.position = self._controller.position
            response.effort = self._controller.torque
        except Exception as e:
            response.success = False
            response.message = str(e)
        return response

    def _on_open_gripper(self, request, response):
        try:
            self._controller.open_gripper()
            response.success = True
            response.message = 'Opening gripper'
        except Exception as e:
            response.success = False
            response.message = str(e)
        return response

    def _on_close_gripper(self, request, response):
        try:
            hold_torque = self.get_parameter('hold_torque').value
            self._controller.close_gripper(hold_torque)
            response.success = True
            response.message = 'Closing gripper'
        except Exception as e:
            response.success = False
            response.message = str(e)
        return response

    def _on_reconnect(self, request, response):
        try:
            self._driver.close()
            self._connect()
            if self._driver.is_open:
                response.success = True
                response.message = 'Reconnected successfully'
            else:
                response.success = False
                response.message = 'Reconnection failed'
        except Exception as e:
            response.success = False
            response.message = str(e)
        return response

    def _control_loop(self):
        try:
            self._controller.update()
        except Exception as e:
            self.get_logger().error(f'Control loop error: {e}')
            return

        js = JointState()
        js.header.stamp = self.get_clock().now().to_msg()
        js.name = [self._joint_name]
        js.position = [self._controller.position]
        js.velocity = [self._controller.velocity]
        js.effort = [self._controller.torque]
        self._pub_joint_state.publish(js)

    def destroy_node(self):
        self.get_logger().info('Shutting down...')
        try:
            self._driver.close()
        except Exception:
            pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = GripperNode()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
