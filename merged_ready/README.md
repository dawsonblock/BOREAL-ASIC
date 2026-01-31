# RFSN E2B Agent

**RFSN Deterministic Agent Kernel** - A safety-first autonomous code repair system with strict invariants.

[![Python 3.9+](https://img.shields.io/badge/python-3.9+-blue.svg)](https://www.python.org/downloads/)
[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](https://opensource.org/licenses/MIT)

## Overview

RFSN (Robust Fail-Safe Network) is a deterministic agent kernel for autonomous software repair. It enforces 7 non-negotiable invariants that guarantee safety and auditability:

1. **Planner never executes** - Planning is read-only
2. **Gate never learns** - Safety rules are immutable
3. **Controller never decides** - Execution requires gate approval
4. **All commits are serial** - One action at a time
5. **No hidden state** - All state is explicit and hashable
6. **All side effects logged** - Complete audit trail
7. **Rejections produce evidence** - Every "no" is justified

## Architecture

```
┌─────────────────────────────────────────────────────────────┐
│                    Upstream Learner                         │
│  ┌──────────┐  ┌─────────────┐  ┌────────────────────────┐ │
│  │  Bandit  │  │ Fingerprint │  │ Memory / Retrieval     │ │
│  │(Thompson)│  │ (Classify)  │  │ (Context)              │ │
│  └────┬─────┘  └──────┬──────┘  └───────────┬────────────┘ │
└───────┼───────────────┼─────────────────────┼──────────────┘
        │               │                     │
        ▼               ▼                     ▼
┌─────────────────────────────────────────────────────────────┐
│                      RFSN Kernel                            │
│  ┌──────────┐    ┌──────┐    ┌────────────┐   ┌──────────┐ │
│  │  Planner │───▶│ Gate │───▶│ Controller │◀──│  Ledger  │ │
│  │(Propose) │    │(Pure)│    │ (Execute)  │   │ (Log)    │ │
│  └──────────┘    └──────┘    └────────────┘   └──────────┘ │
└─────────────────────────────────────────────────────────────┘
```

## Quick Start

```bash
# Clone the repository
git clone https://github.com/dawsonblock/RFSN-E2B-AGENT.git
cd RFSN-E2B-AGENT

# Install dependencies
pip install -e .

# Run an agent task
python run_agent.py \
    --task-id BUG_001 \
    --workspace ./your_repo \
    --task "Fix the failing test in utils.py" \
    --provider deepseek
```

## Packages

### `rfsn_kernel/` - Core Deterministic Kernel

| Module | Purpose |
|--------|---------|
| `state.py` | Immutable state model with SHA256 hashing |
| `proposal.py` | Proposal schema with shell injection detection |
| `gate.py` | Pure deterministic safety gate |
| `controller.py` | Sandboxed executor (requires gate approval) |
| `ledger.py` | Ground truth JSONL logging |
| `kernel.py` | Main orchestration loop |
| `llm_planner.py` | DeepSeek/OpenAI/Anthropic integration |

### `rfsn_upstream/` - Learning Layer (Separate from Kernel)

| Module | Purpose |
|--------|---------|
| `bandit.py` | Thompson Sampling with SQLite persistence |
| `fingerprint.py` | Failure classification & similarity |
| `retrieval.py` | Memory storage with FTS search |
| `prompt_variants.py` | 7 SWE-bench optimized prompts |

## Usage Examples

### Basic Task

```python
from rfsn_kernel import run_kernel, KernelConfig
from rfsn_kernel.llm_planner import LLMPlanner, LLMPlannerConfig

# Configure planner
planner = LLMPlanner(LLMPlannerConfig(
    provider="deepseek",
    model="deepseek-chat"
))

# Run kernel
result = run_kernel(
    task_id="fix_bug_123",
    task="Fix the TypeError in process_data()",
    workspace_path="./my_repo",
    planner=planner,
)

print(f"Success: {result.success}")
print(f"Steps: {result.total_steps}")
```

### With Bandit Learning

```bash
python run_agent.py \
    --task-id BUG_001 \
    --workspace ./repo \
    --task "Fix the bug" \
    --bandit-db ./bandit.db \
    --provider deepseek
```

### SWE-bench Episode

```bash
python scripts/run_swebench_episode.py \
    --instance-id django__django-12345 \
    --repo ./django \
    --arm-id v_minimal_fix \
    --output outcome.json
```

## Environment Variables

```bash
DEEPSEEK_API_KEY=...   # DeepSeek API key
OPENAI_API_KEY=...     # OpenAI API key (optional)
ANTHROPIC_API_KEY=...  # Anthropic API key (optional)
```

## Prompt Variants

The upstream learner includes 7 SWE-bench optimized prompts:

| Variant | Strategy |
|---------|----------|
| `v_minimal_fix` | Single-line minimal changes |
| `v_diagnose_then_patch` | Root cause analysis first |
| `v_test_first` | Understand test expectations |
| `v_multi_hypothesis` | Generate 3 hypotheses, pick best |
| `v_repair_loop` | Learn from rejection feedback |
| `v_context_aware` | Use retrieved similar bugs |
| `v_chain_of_thought` | Explicit reasoning chain |

## CI/CD

The included GitHub Actions workflow (`.github/workflows/swebench_upstream.yml`) automates:

1. Bandit arm selection via Thompson Sampling
2. SWE-bench episode execution
3. Outcome recording and bandit updates
4. Artifact persistence for learning

## License

MIT License - see [LICENSE](LICENSE) for details.
