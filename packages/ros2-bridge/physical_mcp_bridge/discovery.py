"""ROS2 node/topic/service/action discovery via rclpy."""

import rclpy
from rclpy.node import Node


class DiscoveryHandler:
    """Wraps rclpy discovery APIs."""

    def __init__(self, node: Node):
        self.node = node

    def list_topics(self) -> list[dict]:
        """List all topics with their types."""
        topics = self.node.get_topic_names_and_types()
        return [
            {'name': name, 'types': types}
            for name, types in topics
        ]

    def get_topic_info(self, topic: str) -> dict:
        """Get info about a specific topic."""
        publishers = self.node.count_publishers(topic)
        subscribers = self.node.count_subscribers(topic)
        # Find type from topic list
        topic_type = ''
        for name, types in self.node.get_topic_names_and_types():
            if name == topic:
                topic_type = types[0] if types else ''
                break
        return {
            'name': topic,
            'type': topic_type,
            'publishers': publishers,
            'subscribers': subscribers,
        }

    def list_services(self) -> list[dict]:
        """List all services with their types."""
        services = self.node.get_service_names_and_types()
        return [
            {'name': name, 'types': types}
            for name, types in services
        ]

    def get_service_info(self, service: str) -> dict:
        """Get info about a specific service."""
        service_type = ''
        for name, types in self.node.get_service_names_and_types():
            if name == service:
                service_type = types[0] if types else ''
                break
        return {
            'name': service,
            'type': service_type,
        }

    def list_actions(self) -> list[dict]:
        """List all action servers."""
        # Actions appear as a set of topics with specific suffixes
        topics = self.node.get_topic_names_and_types()
        action_names = set()
        for name, _ in topics:
            if name.endswith('/_action/feedback'):
                action_name = name.replace('/_action/feedback', '')
                action_names.add(action_name)
        return [{'name': name} for name in sorted(action_names)]

    def list_nodes(self) -> list[dict]:
        """List all active nodes."""
        nodes = self.node.get_node_names_and_namespaces()
        return [
            {'name': name, 'namespace': ns}
            for name, ns in nodes
        ]
