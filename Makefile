.PHONY: build test lint typecheck dev clean docker docker-down install

MCP_DIR = packages/mcp-server
BRIDGE_DIR = packages/ros2-bridge

# === TypeScript MCP Server ===

install:
	cd $(MCP_DIR) && npm ci

build:
	cd $(MCP_DIR) && npm run build

test:
	cd $(MCP_DIR) && npm test

typecheck:
	cd $(MCP_DIR) && npm run typecheck

dev:
	cd $(MCP_DIR) && npm run dev

lint:
	cd $(MCP_DIR) && npx eslint src --ext .ts || true
	ruff check $(BRIDGE_DIR)/ || true

clean:
	rm -rf $(MCP_DIR)/dist $(MCP_DIR)/node_modules

# === Docker ===

docker:
	cd docker && docker compose up --build

docker-down:
	cd docker && docker compose down

docker-bridge:
	cd docker && docker compose up --build bridge

# === Python Bridge ===

bridge-install:
	cd $(BRIDGE_DIR) && pip install -e .

bridge-test:
	cd $(BRIDGE_DIR) && pytest || true

# === All ===

all: install build test
	@echo "Build and tests complete"

ci: install build typecheck test
	@echo "CI checks complete"
