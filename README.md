# GEARS_Team-41

## Setup

This project uses [UV](https://github.com/astral-sh/uv) for Python package management.

### Installation

1. Install UV:
```bash
curl -LsSf https://astral.sh/uv/install.sh | sh
```

2. Install dependencies:
```bash
uv sync
```

3. Install development dependencies:
```bash
uv sync --extra dev
```

### Development

Run tests:
```bash
uv run pytest
```

Run linter:
```bash
uv run ruff check .
```

Run type checker:
```bash
uv run mypy .
```
