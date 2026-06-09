# pyrefly: ignore [missing-import]
import rclpy
# pyrefly: ignore [missing-import]
from rclpy.node import Node
# pyrefly: ignore [missing-import]
from std_srvs.srv import Trigger
# pyrefly: ignore [missing-import]
from ros_robot_controller_msgs.msg import ServoPosition, ServosPosition
# pyrefly: ignore [missing-import]
from std_msgs.msg import String
# pyrefly: ignore [missing-import]
from openai import OpenAI
import json

class VoiceArmController(Node):
    def __init__(self):
        super().__init__('voice_arm_controller')
        self.pub = self.create_publisher(ServosPosition, '/ros_robot_controller/bus_servo/set_position', 1)
        
        # Cấu hình kết nối tới LLM Local (Qwen qua Ollama)
        # Giả định Ollama đang chạy ở cổng mặc định 11434
        self.get_logger().info('Đang kết nối tới mô hình AI Qwen Local...')
        self.llm_client = OpenAI(
            base_url='http://localhost:11434/v1',
            api_key='ollama', # Ollama không yêu cầu API key thực
        )
        self.model_name = "qwen:0.5b" # Bạn có thể đổi sang "qwen2" hoặc phiên bản bạn đã pull trong Ollama
        
        self.system_prompt = """
        Bạn là trung tâm điều khiển suy luận của một cánh tay robot (ArmPi-Ultra).
        Nhiệm vụ của bạn là phân tích câu lệnh giọng nói ngôn ngữ tự nhiên của người dùng và chuyển đổi nó thành một hành động cụ thể theo định dạng JSON.
        Các hành động (action) được hỗ trợ:
        - "home": Về vị trí ban đầu (thu lại, nghỉ ngơi).
        - "left": Quay toàn bộ cánh tay sang bên trái.
        - "right": Quay toàn bộ cánh tay sang bên phải.
        - "open": Mở kẹp (gripper) ra, thả đồ vật.
        - "close": Đóng kẹp (gripper) lại, gắp đồ vật.
        - "nod": Gật đầu chào.
        - "unknown": Khi câu nói không mang ý nghĩa điều khiển robot, hoặc không liên quan đến các hành động trên.
        
        Chỉ trả về duy nhất chuỗi JSON có định dạng: {"action": "tên_action"}. Không giải thích, không thêm bất kỳ văn bản nào khác.
        """
        
        # Đợi dịch vụ điều khiển cơ bản khởi động
        self.get_logger().info('Đang chờ dịch vụ điều khiển phần cứng của robot...')
        self.client = self.create_client(Trigger, '/ros_robot_controller/init_finish')
        self.client.wait_for_service()
        self.get_logger().info('Sẵn sàng nhận lệnh giọng nói qua AI (Ready for voice commands)...')

        self.voice_sub = self.create_subscription(
            String,
            '/voice_words',
            self.voice_command_callback,
            10
        )

    def set_servo_position(self, duration, positions):
        """
        Gửi lệnh điều khiển một hoặc nhiều servo
        """
        msg = ServosPosition()
        msg.duration = float(duration)
        position_list = []
        for i in positions:
            position = ServoPosition()
            position.id = i[0]
            position.position = int(i[1])
            position_list.append(position)
        msg.position = position_list
        self.pub.publish(msg)
        self.get_logger().info(f'Published servo positions: {positions}')

    def voice_command_callback(self, msg):
        raw_text = msg.data.strip()
        self.get_logger().info(f'Nhận văn bản từ giọng nói: "{raw_text}"')
        
        if not raw_text:
            return
            
        try:
            # Gửi text tới mô hình Qwen Local
            response = self.llm_client.chat.completions.create(
                model=self.model_name,
                messages=[
                    {"role": "system", "content": self.system_prompt},
                    {"role": "user", "content": raw_text}
                ],
                temperature=0.0, # Giảm tính ngẫu nhiên để xuất JSON chuẩn và nhất quán
            )
            
            # Lấy chuỗi phản hồi từ AI
            response_text = response.choices[0].message.content.strip()
            self.get_logger().info(f'Phản hồi từ Qwen AI: {response_text}')
            
            # Làm sạch dữ liệu nếu AI vô tình sinh ra markdown block
            if response_text.startswith("```json"):
                response_text = response_text[7:-3].strip()
            elif response_text.startswith("```"):
                response_text = response_text[3:-3].strip()
                
            # Phân tích cú pháp JSON
            intent = json.loads(response_text)
            action = intent.get("action", "unknown")
            
            # Chạy hành động
            self.execute_action(action)
            
        except json.JSONDecodeError:
            self.get_logger().error(f"Lỗi: AI trả về định dạng không phải JSON hợp lệ. Response: {response_text}")
        except Exception as e:
            self.get_logger().error(f"Lỗi khi kết nối hoặc xử lý LLM: {e}")

    def execute_action(self, action):
        self.get_logger().info(f'Thực thi hành động: [{action}]')
        
        if action == "home":
            self.set_servo_position(1.0, ((1, 500), (2, 500), (3, 500), (4, 500), (5, 500), (6, 500)))
        elif action == "left":
            self.set_servo_position(1.0, ((6, 200),))
        elif action == "right":
            self.set_servo_position(1.0, ((6, 800),))
        elif action == "open":
            self.set_servo_position(0.5, ((1, 200),))
        elif action == "close":
            self.set_servo_position(0.5, ((1, 500),))
        elif action == "nod":
            self.set_servo_position(0.5, ((4, 300),))
        else:
            self.get_logger().info('Không có hành động nào được ánh xạ cho ý định này.')

def main(args=None):
    rclpy.init(args=args)
    node = VoiceArmController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
