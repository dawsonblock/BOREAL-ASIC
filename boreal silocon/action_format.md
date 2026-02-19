# Boreal Action Request/Response Format

## Action Request Format

Action requests are submitted to the Gate via a 64-byte (512-bit) mailbox entry. All fields are little-endian.

### Structure Definition

```c
struct action_req {
    uint32_t opcode;        // Offset 0x00
    uint32_t target;        // Offset 0x04
    uint32_t arg0;          // Offset 0x08
    uint32_t arg1;          // Offset 0x0C
    uint32_t context_hash;  // Offset 0x10
    uint32_t policy_hash;   // Offset 0x14
    uint32_t bounds;        // Offset 0x18
    uint32_t nonce;         // Offset 0x1C
    uint8_t  reserved[32];  // Offset 0x20-0x3F
};
```

### Field Descriptions

| Field | Size | Description |
|-------|------|-------------|
| opcode | 32 bits | Action type identifier |
| target | 32 bits | Register or endpoint ID |
| arg0 | 32 bits | Primary value/length/flags |
| arg1 | 32 bits | Secondary argument |
| context_hash | 32 bits | Hash of input state |
| policy_hash | 32 bits | Expected policy identifier |
| bounds | 32 bits | Clamp/rate class reference |
| nonce | 32 bits | Monotonic sequence number |
| reserved | 256 bits | Future expansion (must be 0) |

### Opcode Definitions

| Value | Name | Description |
|-------|------|-------------|
| 0x0000_0000 | ACT_NOP | No operation (heartbeat) |
| 0x0000_0001 | ACT_WRITE | Write to actuator register |
| 0x0000_0002 | ACT_NET_TX | Enable network transmission |
| 0x0000_0003 | ACT_FLASH | Flash program/erase |
| 0x0000_0004 | ACT_CONFIG | Update configuration |

---

## Action Response Format

Responses are returned via a 20-byte (160-bit) mailbox entry.

### Structure Definition

```c
struct action_resp {
    uint32_t committed;     // Offset 0x00
    uint32_t reason;        // Offset 0x04
    uint32_t applied0;      // Offset 0x08
    uint32_t applied1;      // Offset 0x0C
    uint32_t ledger_idx;    // Offset 0x10
};
```

### Field Descriptions

| Field | Size | Description |
|-------|------|-------------|
| committed | 32 bits | Boolean: 1 if committed, 0 if rejected |
| reason | 32 bits | Status code explaining result |
| applied0 | 32 bits | Sanitized value actually applied |
| applied1 | 32 bits | Secondary sanitized value |
| ledger_idx | 32 bits | Ledger entry index for this decision |

### Reason Codes

| Value | Name | Description |
|-------|------|-------------|
| 0x0000_0000 | OK | Action committed successfully |
| 0x0000_0001 | CLAMPED | Value clamped to bounds |
| 0x0000_0002 | RATE_LIMITED | Rate limit exceeded |
| 0x0000_0003 | POLICY_DENIED | Target not in allowlist |
| 0x0000_0004 | INVALID | Invalid request format |
| 0x0000_0005 | NONCE_ERROR | Nonce check failed |

---

## Ledger Event Format

Each ledger entry is 256 bits (32 bytes).

### Structure Definition

```c
struct ledger_event {
    uint32_t cycle;           // [255:224] Cycle counter
    uint32_t nonce;           // [223:192] Request nonce
    uint32_t opcode;          // [191:160] Action opcode
    uint32_t target;          // [159:128] Target ID
    uint32_t arg0_sanitized;  // [127:96]  Sanitized arg0
    uint32_t committed;       // [95:64]   Committed flag + reason
    uint32_t context_hash;    // [63:32]   Context hash
    uint32_t policy_hash;     // [31:0]    Policy hash
};
```

### Hash Chain

Each event is chained to the previous event:

```
running_hash = SHA256(prev_hash || event_body)
```

This creates a tamper-evident audit trail.

---

## Usage Example

### Submitting an Action Request

```c
// Construct action request
struct action_req req = {
    .opcode = ACT_WRITE,
    .target = 0x0001,      // Actuator 1
    .arg0 = 500,           // Value to write
    .arg1 = 0,
    .context_hash = compute_context_hash(),
    .policy_hash = current_policy_hash,
    .bounds = 0,           // Use default clamps
    .nonce = next_nonce++
};

// Write to mailbox
write_mailbox(ACTION_MB_BASE, &req, sizeof(req));

// Poll for response
struct action_resp resp;
while (!read_mailbox(ACTION_MB_BASE + 4, &resp, sizeof(resp)));

if (resp.committed) {
    printf("Action committed at ledger index %d\n", resp.ledger_idx);
} else {
    printf("Action rejected: reason %d\n", resp.reason);
}
```
