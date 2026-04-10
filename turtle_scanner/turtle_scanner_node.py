import rclpy
from rclpy.node import Node
from turtlesim.msg import Pose
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool        # nouveau import pour la détection
import math

class TurtleScannerNode(Node):
    def __init__(self):
        super().__init__('turtle_scanner_node')

        # ── Variables de position ──────────────────────────
        self.pose_scanner = None
        self.pose_target  = None

        # ── Paramètres du serpentin ────────────────────────
        self.Kp_ang             = 4.0
        self.Kp_lin             = 1.5
        self.linear_speed_max   = 2.0
        self.waypoint_tolerance = 0.3

        # ── Paramètre de détection ─────────────────────────
        # Si la tortue scanner est à moins de 1.5 unités
        # de turtle_target → cible détectée !
        self.detection_radius = 1.5

        # Ce booléen évite de détecter plusieurs fois
        # True = déjà détecté, on ne re-détecte plus
        self.detected = False

        # ── Génération des waypoints du serpentin ──────────
        NB_LIGNES    = 5
        Y_START      = 1.0
        Y_STEP       = 2.0
        X_MIN, X_MAX = 1.0, 10.0

        self.waypoints = []
        for i in range(NB_LIGNES):
            y = Y_START + i * Y_STEP
            x = X_MAX if i % 2 == 0 else X_MIN
            self.waypoints.append((x, y))

        self.wp_idx = 0

        self.get_logger().info(f'Waypoints : {self.waypoints}')

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

        # ── Publisher détection ────────────────────────────
        # Ce publisher envoie True ou False sur /target_detected
        # True  = cible trouvée
        # False = cible pas encore trouvée
        self.pub_detected = self.create_publisher(
            Bool,
            '/target_detected',
            10
        )

        # ── Timer 20 Hz ────────────────────────────────────
        self.create_timer(0.05, self.scan_step)

        self.get_logger().info('Nœud scanner démarré !')

    # ────────────────────────────────────────────────────────
    # Callbacks subscribers
    # ────────────────────────────────────────────────────────

    def cb_scanner(self, msg):
        # Appelé automatiquement quand turtle1 change de position
        self.pose_scanner = msg

    def cb_target(self, msg):
        # Appelé automatiquement quand turtle_target change de position
        self.pose_target = msg

    # ────────────────────────────────────────────────────────
    # Méthodes de calcul
    # ────────────────────────────────────────────────────────

    def compute_angle(self, A, B):
        """Retourne l'angle en radians de A vers B"""
        return math.atan2(B[1] - A[1], B[0] - A[0])

    def compute_distance(self, A, B):
        """Retourne la distance euclidienne entre A et B"""
        return math.sqrt((B[0] - A[0])**2 + (B[1] - A[1])**2)

    # ────────────────────────────────────────────────────────
    # Méthode principale appelée 20x par seconde
    # ────────────────────────────────────────────────────────

    def scan_step(self):

        # Sécurité : si on n'a pas encore reçu la position → attendre
        if self.pose_scanner is None:
            return

        # Position actuelle de turtle1
        A = (self.pose_scanner.x, self.pose_scanner.y)

        # ════════════════════════════════════════════════════
        # BLOC 1 : DÉTECTION DE LA CIBLE
        # Ce bloc est vérifié EN PREMIER, avant tout le reste
        # ════════════════════════════════════════════════════

        if self.pose_target is not None and not self.detected:
            # Position de la cible
            T = (self.pose_target.x, self.pose_target.y)

            # Calculer la distance entre le scanner et la cible
            dist_cible = self.compute_distance(A, T)

            if dist_cible < self.detection_radius:
                # ── CIBLE TROUVÉE ! ──────────────────────
                self.detected = True  # ne plus re-détecter

                # Arrêter immédiatement la tortue
                # Twist() sans valeur = vitesse 0 partout
                self.pub_cmd.publish(Twist())

                # Publier True sur /target_detected
                msg_detected = Bool()
                msg_detected.data = True
                self.pub_detected.publish(msg_detected)

                # Afficher dans le terminal
                self.get_logger().info(
                    f'Cible détectée à ({T[0]:.2f}, {T[1]:.2f}) !'
                )
                return  # sortir de scan_step, ne pas continuer

        # ════════════════════════════════════════════════════
        # BLOC 2 : PUBLIER FALSE SI PAS ENCORE DÉTECTÉE
        # On publie False en permanence tant que pas détecté
        # ════════════════════════════════════════════════════

        if not self.detected:
            msg_not_detected = Bool()
            msg_not_detected.data = False
            self.pub_detected.publish(msg_not_detected)

        # ════════════════════════════════════════════════════
        # BLOC 3 : BALAYAGE EN SERPENTIN (code de la Partie 3)
        # Ce bloc ne s'exécute QUE si la cible n'est pas détectée
        # ════════════════════════════════════════════════════

        # Si déjà détecté → ne plus bouger
        if self.detected:
            return

        # Si tous les waypoints parcourus → arrêter
        if self.wp_idx >= len(self.waypoints):
            return

        # Waypoint cible
        B = self.waypoints[self.wp_idx]

        dist = self.compute_distance(A, B)

        # Waypoint atteint ?
        if dist < self.waypoint_tolerance:
            self.get_logger().info(
                f'Waypoint {self.wp_idx} atteint ! '
                f'({B[0]:.1f}, {B[1]:.1f})'
            )
            self.wp_idx += 1

            if self.wp_idx >= len(self.waypoints):
                self.pub_cmd.publish(Twist())
                self.get_logger().info('Balayage terminé sans détection.')
            return

        # Calculer et publier la commande de vitesse
        theta_desired = self.compute_angle(A, B)
        e = math.atan(
            math.tan((theta_desired - self.pose_scanner.theta) / 2)
        )

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
