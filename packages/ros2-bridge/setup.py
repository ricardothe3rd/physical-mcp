from setuptools import setup, find_packages

setup(
    name='physical-mcp-bridge',
    version='0.1.0',
    packages=find_packages(),
    install_requires=[
        'websockets>=12.0',
    ],
    entry_points={
        'console_scripts': [
            'physical-mcp-bridge=physical_mcp_bridge.bridge_node:main',
        ],
    },
    python_requires='>=3.10',
)
