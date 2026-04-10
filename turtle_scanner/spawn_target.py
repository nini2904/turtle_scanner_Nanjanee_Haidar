import rclpy                          # bibliothèque principale ROS 2
from rclpy.node import Node           # classe de base pour un nœud
from turtlesim.srv import Spawn       # type du service /spawn
import random                          # pour les nombres aléatoires

class SpawnTarget(Node):
    def __init__(self):
        super().__init__('spawn_target')  # nom du nœud
        
        # Créer un CLIENT de service (côté qui demande)
        self.client = self.create_client(Spawn, '/spawn')
        
        # Attendre que le service soit disponible
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Attente du service /spawn...')
        
        self.spawn_turtle()  # appeler la fonction de spawn

    def spawn_turtle(self):
        req = Spawn.Request()           # créer la requête
        req.x = random.uniform(1.0, 10.0)  # position X aléatoire
        req.y = random.uniform(1.0, 10.0)  # position Y aléatoire
        req.theta = 0.0                 # orientation
        req.name = 'turtle_target'      # nom de la tortue
        
        # Envoyer la requête et attendre la réponse
        future = self.client.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        
        self.get_logger().info(
            f'Cible spawnée à x={req.x:.2f}, y={req.y:.2f}'
        )

def main():
    rclpy.init()           # initialiser ROS 2
    node = SpawnTarget()   # créer le nœud
    rclpy.shutdown()       # fermer ROS 2

if __name__ == '__main__':
    main()
