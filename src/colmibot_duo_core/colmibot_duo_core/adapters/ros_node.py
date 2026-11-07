"""
Adaptador ROS 2 mínimo: recibe comandos JSON y publica el estado de la misión.

Interfaz (nombres relativos al nodo, de modo que un namespace por robot los aísla):
- suscripción  ~/command  std_msgs/String  (JSON, ver command_dispatch)
- publicación  ~/state    std_msgs/String  (JSON: respuesta y resumen de la misión)

Es el único módulo que importa ROS. No publica ningún otro tipo de mensaje y no abre
dispositivos ni conexiones: la evidencia se guarda en un archivo SQLite local.
"""

from datetime import datetime, timezone
import json
from pathlib import Path

from colmibot_duo_core.adapters.command_dispatch import dispatch
from colmibot_duo_core.domain.route import StationCatalog
from colmibot_duo_core.persistence.sqlite_store import EvidenceStore
from colmibot_duo_core.service import MissionService
from rcl_interfaces.msg import ParameterDescriptor
import rclpy
from rclpy.executors import ExternalShutdownException, SingleThreadedExecutor
from rclpy.logging import get_logger
from rclpy.node import Node
from std_msgs.msg import String


def _utc_now() -> datetime:
    return datetime.now(timezone.utc)


class MissionNode(Node):
    """Nodo de misión de un robot. Parámetros: robot_id, db_dir, stations, origin."""

    def __init__(self, **node_kwargs):
        """Crea el nodo y recupera la misión persistida, si la hay."""
        super().__init__('mission_node', **node_kwargs)
        # Se leen una sola vez al arrancar: cambiarlos en caliente no cambiaría la identidad
        # ni el archivo de evidencia en uso, así que ROS debe rechazar el cambio.
        fixed = ParameterDescriptor(read_only=True)
        self.declare_parameter('robot_id', 'A', fixed)
        self.declare_parameter('db_dir', '', fixed)
        self.declare_parameter('stations', ['S1', 'S2', 'S3'], fixed)
        self.declare_parameter('origin', 'synthetic', fixed)
        robot_id = self.get_parameter('robot_id').value
        db_dir = self.get_parameter('db_dir').value
        if not db_dir:
            raise ValueError('el parámetro db_dir es obligatorio: directorio fuera del '
                             'código fuente para el archivo de evidencia')
        Path(db_dir).mkdir(parents=True, exist_ok=True)
        self._store = EvidenceStore(Path(db_dir) / f'evidence_{robot_id}.sqlite3', robot_id)
        self._service = MissionService(
            self._store, StationCatalog(self.get_parameter('stations').value), _utc_now,
            origin=self.get_parameter('origin').value)
        self._service.recover()
        self._state_pub = self.create_publisher(String, '~/state', 10)
        self.create_subscription(String, '~/command', self._on_command, 10)
        state = self._service.snapshot()['state']
        self.get_logger().info(f'mission_node listo: robot {robot_id}, estado {state}')

    def destroy_node(self):
        """Cierra el archivo de evidencia y destruye el nodo."""
        self._store.close()
        return super().destroy_node()

    def _on_command(self, msg: String) -> None:
        try:
            message = json.loads(msg.data)
        except (ValueError, RecursionError):  # RecursionError: JSON con anidación excesiva
            reply = {'ok': False, 'error': 'ValidationError', 'detail': 'JSON inválido'}
        else:
            try:
                reply = dispatch(self._service, message)
            except Exception as exc:  # un comando mal formado no debe tumbar el nodo
                self.get_logger().error(f'error inesperado al despachar: {exc!r}')
                reply = {'ok': False, 'error': 'InternalError', 'detail': type(exc).__name__}
        out = String()
        out.data = json.dumps({'reply': reply, 'snapshot': self._service.snapshot()})
        self._state_pub.publish(out)


def main(args=None):
    """
    Punto de entrada del ejecutable mission_node.

    Un apagado pedido desde fuera (SIGINT, SIGTERM o rclpy.shutdown) es una salida normal:
    se cierra el archivo de evidencia, se libera el contexto y el proceso termina con código 0.
    Un fallo real (por ejemplo, un parámetro obligatorio ausente) sí se propaga.
    """
    rclpy.init(args=args)
    node = None
    executor = None
    try:
        node = MissionNode()
        executor = SingleThreadedExecutor()
        executor.add_node(node)
        executor.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    except Exception as exc:
        if rclpy.ok():
            raise
        # El contexto ya fue apagado desde fuera (señal durante el arranque): no es un fallo.
        get_logger('mission_node').warning(f'apagado durante el arranque: {exc!r}')
    finally:
        if executor is not None:
            executor.shutdown()
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
