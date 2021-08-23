#ifndef _COBS_H
#define _COBS_H

// COBS codec.
// Supports up to 250 bytes of payload.

size_t COBSEncode(const uint8_t * from, const size_t length, uint8_t * dst, const size_t dst_length) {
    size_t from_ptr = 0; // Next byte to be read from "from" in loop
    size_t dst_ptr = 1; // Next byte to be written to "dst" in loop
    size_t code_ptr = 0; // Where to write "code" in "dst"
    uint8_t code = 1;
    while ((dst_ptr + 1) < dst_length) {
        uint8_t in_byte = from[from_ptr];
        from_ptr ++;
        if (in_byte == 0) {
            dst[code_ptr] = code;
            // Reset code
            code = 1;
            code_ptr = dst_ptr;
            dst_ptr ++;
        } else {
            dst[dst_ptr] = in_byte;
            dst_ptr ++;
            // Special case where code = 255 is not supported
            code ++;
        }
        if (from_ptr >= length) {
            dst[code_ptr] = code;
            dst[dst_ptr] = 0;
            // Return length with terminating 0
            return dst_ptr + 1;
        }
    }
    // Something is broken, tell dst_length is 0
    return 0;
}

size_t COBSDecode(const uint8_t * encoded, const size_t length, uint8_t * dst, const size_t dst_length) {
    // "encoded" contains only one frame and ends with 0
    if (encoded[length - 1] != 0) {
        // Bad input
        return 0;
    }
    size_t in_ptr = 0; // Next byte to read
    size_t out_ptr = 0; // Next byte to write
    while (in_ptr < length) {
        dst[out_ptr] = 0;
        out_ptr ++;
        if (encoded[in_ptr] == 0) {
          // 0 should never appear in data
          return 0;
        }
        size_t to_copy = encoded[in_ptr] - 1;
        for (size_t i = 0; i < to_copy; i++) {
            dst[out_ptr] = encoded[in_ptr + 1 + i];
            out_ptr ++;
        }
        in_ptr += (to_copy + 1);
        if (encoded[in_ptr] == 0) {
            break;
        }
    }
    // Remove leading zero
    if (encoded[0] != 1 && dst[0] == 0) {
        for (size_t i = 1; i < out_ptr; i++) {
            dst[i-1] = dst[i];
        }
        out_ptr --;
    }
    // Return number of bytes written
    return out_ptr;
}

#endif
