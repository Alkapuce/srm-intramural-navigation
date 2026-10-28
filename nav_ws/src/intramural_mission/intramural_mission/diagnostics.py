"""Isolate reliable diagnostic DDS publication from the control process and its GIL."""
import multiprocessing as mp
import threading


def _publish(reader, namespace, domain, ready):
    import signal
    import rclpy
    from rclpy.qos import QoSProfile, DurabilityPolicy
    from rclpy.signals import SignalHandlerOptions
    from std_msgs.msg import String
    from nav_msgs.msg import Path
    from geometry_msgs.msg import PoseStamped
    signal.signal(signal.SIGINT, signal.SIG_IGN)
    rclpy.init(domain_id=domain, signal_handler_options=SignalHandlerOptions.NO)
    node = rclpy.create_node('mission_diagnostics', namespace=namespace)
    qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
    publishers = {topic: node.create_publisher(kind, topic, qos)
                  for topic, kind in (('mission/status', String), ('mission/path', Path))}
    ready.set()
    try:
        while True:
            packet = reader.recv()
            if packet is None:
                break
            topic, serialized = packet
            if topic=='mission/path' and isinstance(serialized,tuple):
                points,sec,nanosec=serialized
                message=Path();message.header.frame_id='map'
                message.header.stamp.sec=sec;message.header.stamp.nanosec=nanosec
                for x,y in points:
                    pose=PoseStamped();pose.header=message.header
                    pose.pose.position.x=x;pose.pose.position.y=y;pose.pose.orientation.w=1.
                    message.poses.append(pose)
                serialized=message
            publishers[topic].publish(serialized)
    except EOFError:
        pass
    finally:
        reader.close()
        node.destroy_node()
        rclpy.try_shutdown()


class Diagnostics:
    def __init__(self, namespace, domain):
        context = mp.get_context('spawn')
        reader, self.writer = context.Pipe(duplex=False)
        self.ready = context.Event()
        self.condition = threading.Condition()
        self.pending, self.closing, self.error = {}, False, None
        self.process = context.Process(target=_publish, args=(reader, namespace, domain, self.ready), daemon=True)
        self.process.start()
        reader.close()
        self.thread = threading.Thread(target=self.send, daemon=True, name='mission-diagnostics')
        self.thread.start()

    def check(self):
        if self.error is not None:
            raise RuntimeError('Diagnostic sender failed') from self.error
        if not self.closing and not self.process.is_alive():
            raise RuntimeError('Diagnostic publisher exited')

    def publish(self, topic, message):
        from rclpy.serialization import serialize_message
        serialized = serialize_message(message)
        self.enqueue(topic,serialized)

    def publish_path(self,points,stamp):
        self.enqueue('mission/path',(points,stamp.sec,stamp.nanosec))

    def enqueue(self,topic,serialized):
        self.check()
        with self.condition:
            if self.closing:
                raise RuntimeError('Diagnostic publisher closed')
            self.pending[topic] = serialized
            self.condition.notify()

    def send(self):
        try:
            while True:
                with self.condition:
                    self.condition.wait_for(lambda: self.pending or self.closing)
                    if not self.pending:
                        self.writer.send(None)
                        return
                    packet = self.pending.popitem()
                self.writer.send(packet)
        except (EOFError, OSError) as error:
            self.error = error
        finally:
            self.writer.close()

    def close(self):
        with self.condition:
            self.closing = True
            self.condition.notify()
        self.thread.join(timeout=2.)
        self.process.join(timeout=2.)
        if self.thread.is_alive() or self.process.is_alive():
            self.process.terminate()
            self.process.join(timeout=1.)
            self.thread.join(timeout=1.)
            raise RuntimeError('Diagnostic publisher did not finish')
        self.check()
        if self.process.exitcode:
            raise RuntimeError(f'Diagnostic publisher exit {self.process.exitcode}')
