import rclpy
from rclpy.node import Node
from turtlesim.msg import Pose
from geometry_msgs.msg import Twist
import math

class TurtleScannerNode(Node):
    def __init__(self):
        super().__init__('turtle_scanner_node')

        # ── Variables de position ──────────────────────────
        self.pose_scanner = None
        self.pose_target  = None

        # ── Paramètres du serpentin ────────────────────────
        self.Kp_ang           = 4.0   # gain angulaire
        self.Kp_lin           = 1.5   # gain linéaire
        self.linear_speed_max = 2.0   # vitesse max
        self.waypoint_tolerance = 0.3 # distance pour considérer waypoint atteint

        # ── Génération des waypoints ───────────────────────
        NB_LIGNES    = 5
        Y_START      = 1.0
        Y_STEP       = 2.0
        X_MIN, X_MAX = 1.0, 10.0

        self.waypoints = []
        for i in range(NB_LIGNES):
            y = Y_START + i * Y_STEP
            x = X_MAX if i % 2 == 0 else X_MIN
            self.waypoints.append((x, y))

        self.wp_idx = 0  # index du waypoint courant

        self.get_logger().info(f'Waypoints générés : {self.waypoints}')

        # ── Subscribers ────────────────────────────────────
        self.create_subscription(
            Pose,
            '/turtle1/pose',
            self.cb_scanner,
            10
        )
        self.create_subscription(
            Pose,
            '/turtle_target/pose',
            self.cb_target,
            10
        )

        # ── Publisher vitesse ──────────────────────────────
        self.pub_cmd = self.create_publisher(
            Twist,
            '/turtle1/cmd_vel',
            10
        )

        # ── Timer 20 Hz ────────────────────────────────────
        self.create_timer(0.05, self.scan_step)

        self.get_logger().info('Nœud scanner démarré ! Balayage en cours...')

    # ── Callbacks subscribers ──────────────────────────────
    def cb_scanner(self, msg):
        self.pose_scanner = msg

    def cb_target(self, msg):
        self.pose_target = msg

    # ── Méthodes de calcul ─────────────────────────────────
    def compute_angle(self, A, B):
        """Calcule l'angle de A vers B"""
        return math.atan2(B[1] - A[1], B[0] - A[0])

    def compute_distance(self, A, B):
        """Calcule la distance entre A et B"""
        return math.sqrt((B[0] - A[0])**2 + (B[1] - A[1])**2)

    # ── Méthode principale du balayage ─────────────────────
    def scan_step(self):

        # Pas encore de position reçue → attendre
        if self.pose_scanner is None:
            return

        # Tous les waypoints parcourus → arrêter
        if self.wp_idx >= len(self.waypoints):
            return

        # Position actuelle et waypoint cible
        A = (self.pose_scanner.x, self.pose_scanner.y)
        B = self.waypoints[self.wp_idx]

        dist = self.compute_distance(A, B)

        # ── Waypoint atteint ? ─────────────────────────────
        if dist < self.waypoint_tolerance:
            self.get_logger().info(
                f'Waypoint {self.wp_idx} atteint ! '
                f'({B[0]:.1f}, {B[1]:.1f})'
            )
            self.wp_idx += 1

            # Dernier waypoint atteint
            if self.wp_idx >= len(self.waypoints):
                self.pub_cmd.publish(Twist())  # vitesse nulle
                self.get_logger().info('Balayage terminé !')
            return

        # ── Calcul de la commande ──────────────────────────
        theta_desired = self.compute_angle(A, B)

        # Erreur angulaire (formule qui évite les sauts -pi/pi)
        e = math.atan(
            math.tan((theta_desired - self.pose_scanner.theta) / 2)
        )

        # Commande proportionnelle
        cmd = Twist()
        cmd.angular.z = self.Kp_ang * e
        cmd.linear.x  = min(self.Kp_lin * dist, self.linear_speed_max)

        self.pub_cmd.publish(cmd)


def main():
    rclpy.init()
    node = TurtleScannerNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
