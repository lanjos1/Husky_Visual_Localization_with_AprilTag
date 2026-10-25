#!/usr/bin/env python3
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

import rospy
import cv2
import numpy as np
import threading
from sensor_msgs.msg import Image, CameraInfo
from gazebo_msgs.msg import ModelStates
from cv_bridge import CvBridge, CvBridgeError
from pupil_apriltags import Detector
from scipy.spatial.transform import Rotation as R

class TagPoseEstimator:
    def __init__(self):
        rospy.init_node('tag_pose_estimator', anonymous=True)
        self.bridge = CvBridge()
        self.lock = threading.Lock()

        # Configuração do detetor AprilTag
        self.at_detector = Detector(
            families='tag36h11',
            nthreads=1,
            quad_decimate=1.0,
            quad_sigma=0.0,
            refine_edges=1,
            decode_sharpening=0.25,
            debug=0
        )

        self.tag_size = 0.15  
        self.camera_matrix = None
        self.dist_coeffs = None

        self.gt_x = None
        self.gt_y = None
        self.gt_yaw = None

        # Posição central da câmara cenital no mundo do Gazebo
        self.cam_x = 6.5
        self.cam_y = 4.0
        
        cam_quat = [0.0, 0.7071080798594737, 0.0, 0.7071054825112364]
        r_cam = R.from_quat(cam_quat)
        self.R_world_cam = r_cam.as_matrix()

        # Listas de histórico
        self.t_log = []
        self.est_x_log = []
        self.est_y_log = []
        self.est_yaw_log = []
        self.gt_x_log = []
        self.gt_y_log = []
        self.gt_yaw_log = []

        rospy.Subscriber('/overhead_camera/camera_info', CameraInfo, self.camera_info_callback)
        rospy.Subscriber('/overhead_camera/image_raw', Image, self.image_callback)
        rospy.Subscriber('/gazebo/model_states', ModelStates, self.gt_callback)

        rospy.on_shutdown(self.plot_trajectory)
        rospy.loginfo("Nó iniciado com cálculo de RMSE. Pressione Ctrl+C para gerar o relatório e gráficos.")

    def camera_info_callback(self, msg):
        if self.camera_matrix is None:
            self.camera_matrix = np.array(msg.K).reshape(3, 3)
            self.dist_coeffs = np.array(msg.D)

    def gt_callback(self, msg):
        try:
            idx = msg.name.index('husky')
            self.gt_x = msg.pose[idx].position.x
            self.gt_y = msg.pose[idx].position.y
            
            orientation = msg.pose[idx].orientation
            q = [orientation.x, orientation.y, orientation.z, orientation.w]
            r_gt = R.from_quat(q)
            self.gt_yaw = r_gt.as_euler('xyz', degrees=True)[2]
        except ValueError:
            pass

    def image_callback(self, msg):
        if self.camera_matrix is None:
            rospy.loginfo_throttle(2.0, "À espera dos parâmetros da câmara (CameraInfo)...")
            return

        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
        except CvBridgeError as e:
            rospy.logerr(f"Erro no CvBridge: {e}")
            return

        fx = self.camera_matrix[0, 0]
        fy = self.camera_matrix[1, 1]
        cx = self.camera_matrix[0, 2]
        cy = self.camera_matrix[1, 2]
        camera_params = (fx, fy, cx, cy)

        tags = self.at_detector.detect(
            gray, 
            estimate_tag_pose=True, 
            camera_params=camera_params, 
            tag_size=self.tag_size
        )

        for tag in tags:
            tag_id = tag.tag_id
            t_vec = tag.pose_t  # Translação [X_cam, Y_cam, Z_cam]
            R_tag_cam = tag.pose_R

            # Projeção geométrica direta do referencial da câmara para o mundo
            dx = t_vec[0][0]
            dy = t_vec[1][0]
            
            robot_x = self.cam_x - dy
            robot_y = self.cam_y - dx

            # Orientação (Yaw) rigorosa com a matriz de rotação combinada
            R_world_tag = self.R_world_cam.dot(R_tag_cam)
            r_estimated = R.from_matrix(R_world_tag)
            raw_yaw = r_estimated.as_euler('zxy', degrees=True)[0]
            
            robot_yaw = -raw_yaw + 180.0  
            robot_yaw = (robot_yaw + 180) % 360 - 180

            if self.gt_x is not None and self.gt_y is not None and self.gt_yaw is not None:
                err_x = abs(robot_x - self.gt_x)
                err_y = abs(robot_y - self.gt_y)
                euclidean_err = np.sqrt(err_x**2 + err_y**2)
                err_yaw = (robot_yaw - self.gt_yaw + 180) % 360 - 180

                with self.lock:
                    current_time = rospy.get_time()
                    self.t_log.append(current_time)
                    self.est_x_log.append(robot_x)
                    self.est_y_log.append(robot_y)
                    self.est_yaw_log.append(robot_yaw)
                    self.gt_x_log.append(self.gt_x)
                    self.gt_y_log.append(self.gt_y)
                    self.gt_yaw_log.append(self.gt_yaw)

                rospy.loginfo(
                    f"Tag ID: {tag_id} | "
                    f"Visão: (X:{robot_x:.2f}m, Y:{robot_y:.2f}m, Yaw:{robot_yaw:.1f}°) | "
                    f"GT: (X:{self.gt_x:.2f}m, Y:{self.gt_y:.2f}m, Yaw:{self.gt_yaw:.1f}°) | "
                    f"Erro Posição: {euclidean_err:.3f}m | Erro Orientação: {abs(err_yaw):.2f}°"
                )

        cv2.imshow("Overhead Tag Detection", cv_image)
        cv2.waitKey(1)

    def plot_trajectory(self):
        """Gera o gráfico, calcula o RMSE global e guarda tudo quando o ROS é encerrado."""
        with self.lock:
            if not self.t_log:
                rospy.logwarn("Nenhum dado de trajetória foi registado para plotar.")
                return

            rospy.loginfo("A gerar relatórios, estatísticas de RMSE e gráficos...")
            
            t_data = list(self.t_log)
            est_x = list(self.est_x_log)
            est_y = list(self.est_y_log)
            est_yaw = list(self.est_yaw_log)
            gt_x = list(self.gt_x_log)
            gt_y = list(self.gt_y_log)
            gt_yaw = list(self.gt_yaw_log)

        # --- CÁLCULO DAS MÉTRICAS DE ERRO (RMSE) ---
        pos_errors = np.sqrt((np.array(est_x) - np.array(gt_x))**2 + (np.array(est_y) - np.array(gt_y))**2)
        rmse_position = np.sqrt(np.mean(pos_errors**2))

        yaw_errors = np.array([abs((ey - gy + 180) % 360 - 180) for ey, gy in zip(est_yaw, gt_yaw)])
        rmse_yaw = np.sqrt(np.mean(yaw_errors**2))

        # Imprime o relatório final estruturado no terminal
        rospy.loginfo("==========================================")
        rospy.loginfo("          RELATÓRIO FINAL - RMSE          ")
        rospy.loginfo("==========================================")
        rospy.loginfo(f" RMSE de Posição (Euclidiano): {rmse_position:.4f} metros")
        rospy.loginfo(f" RMSE de Orientação (Yaw):     {rmse_yaw:.4f} graus")
        rospy.loginfo("==========================================")

        t_start = t_data[0]
        time_elapsed = [t - t_start for t in t_data]

        plt.figure(figsize=(14, 10))

        # 1. Trajetória 2D
        plt.subplot(2, 2, 1)
        plt.plot(gt_x, gt_y, 'b-', label='Ground Truth (Gazebo)', linewidth=2)
        plt.plot(est_x, est_y, 'r--', label='Estimado (Visão/Tag)', linewidth=2)
        plt.title('Trajetória 2D no Plano (X vs Y)')
        plt.xlabel('Posição X (m)')
        plt.ylabel('Posição Y (m)')
        plt.legend()
        plt.grid(True)
        plt.axis('equal')

        # 2. Erro de Posição
        plt.subplot(2, 2, 2)
        plt.plot(time_elapsed, pos_errors, 'g-', linewidth=1.5)
        plt.title(f'Erro de Posição (RMSE: {rmse_position:.3f}m)')
        plt.xlabel('Tempo (s)')
        plt.ylabel('Erro (m)')
        plt.grid(True)

        # 3. Comparação de Orientação (Yaw)
        plt.subplot(2, 2, 3)
        plt.plot(time_elapsed, gt_yaw, 'b-', label='GT Yaw', linewidth=1.5)
        plt.plot(time_elapsed, est_yaw, 'r--', label='Estimado Yaw', linewidth=1.5)
        plt.title('Comparação de Orientação (Yaw)')
        plt.xlabel('Tempo (s)')
        plt.ylabel('Yaw (graus)')
        plt.legend()
        plt.grid(True)

        # 4. Erro de Orientação
        plt.subplot(2, 2, 4)
        plt.plot(time_elapsed, yaw_errors, 'm-', linewidth=1.5)
        plt.title(f'Erro de Orientação (RMSE: {rmse_yaw:.2f}°)')
        plt.xlabel('Tempo (s)')
        plt.ylabel('Erro Yaw (graus)')
        plt.grid(True)

        plt.tight_layout()
        
        output_filename = "resultado_trajetoria_husky.png"
        plt.savefig(output_filename, dpi=300)
        rospy.loginfo(f"Gráfico guardado com sucesso em: {output_filename}")

if __name__ == '__main__':
    try:
        estimator = TagPoseEstimator()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass