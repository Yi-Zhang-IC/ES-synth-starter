# Embedded CW 2 CAN Protocol

## Key event – 1 byte 

```
| 0 | DOWN? | KEY_CODE |
 [7]   [6]     [5:0]
```

## Settings update – 2 bytes

```
| 1 0 | OPTION_ID |, | OPTION_VALUE |
 [7:6]    [5:0]           [7:0]
```

## Handshaking

### Procedure

1. At POR, all modules switch handshaking signals on
2. Special case: If no other modules connected, skip all handshaking
3. Leftmost module detects its position based on the handshaking signals and announces its ID and position index (0)
4. Leftmost module switches its right-side handshaking signal off to signal the next module to announce its ID and position
5. Process continues until the rightmost module has finished announcing
6. Rightmost module broadcasts a “done” message → allows all modules to know how many there are

```
ID and position announcement:
| 1 1 0 | POSITION_INDEX |, | UNIQUE_ID_0 |, | UNIQUE_ID_1 |, | UNIQUE_ID_2 |, | UNIQUE_ID_3 |
  [7:5]       [4:0]              [7:0]            [7:0]            [7:0]            [7:0]
```

```
"Done" announcement:
| 1 1 1 | RESERVED_ZERO |
  [7:5]       [4:0]
```

---

Encoding inspired by the Quite OK Image Format. 
