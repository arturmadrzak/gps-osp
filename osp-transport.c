#include <stdlib.h>
#include <stdbool.h>
#include <stdio.h>
#include "osp-transport.h"
#include "endian.h"

const uint8_t HEADER[] = {0xa0, 0xa2};
const uint8_t TAIL[] = {0xb0, 0xb3};

typedef struct {
    io_t pub;
    io_t *io;
} osp_transport_t;

static uint16_t osp_checksum(uint8_t *payload, uint16_t plen) {
    uint16_t cksum = 0;
    int i;
    for(i = 0; i < plen; i++) {
        cksum += (uint16_t)payload[i];
        cksum &= 0x7FFF;
    }
    return cksum;
}

static int read_exactly(io_t *io, uint8_t *buffer, uint16_t len) {
    int already = 0;
    int rv;
    do {
        if ((rv = io->read(io, &buffer[already], len - already)) <= 0)
            return rv;
        already += rv;
    } while (already != len);
    return already;
}

static int scan_for_header(io_t* io) {
    uint8_t b;
    bool nibble = false;
    do {
        if (io->read(io, &b, 1) != 1)
            return -1;
        if (!nibble && b == 0xa0)
            nibble = true;
        else if (nibble && b == 0xa2)
            break;
        else
            nibble = false;
    } while(1);
    return 0;
}

static int m_open(io_t *io)
{
    osp_transport_t *ot = (osp_transport_t*)io;
    return ot->io->open(ot->io);
}

static int m_write(io_t *io, void *buffer, size_t size)
{
    osp_transport_t *ot = (osp_transport_t*)io;
    io_t *lio = ot->io;
    uint16_t ck_be = htobe16(osp_checksum(buffer, size));
    uint16_t size_be = htobe16((uint16_t)size);
    if (size > 0xFFFF)
        return -1;
    if (lio->write(lio, (uint8_t*)&HEADER, sizeof(HEADER)) < 0) goto osp_send_error;
    if (lio->write(lio, &size_be, 2) < 0) goto osp_send_error;
    if (lio->write(lio, buffer, size) < 0) goto osp_send_error;
    if (lio->write(lio, &ck_be, 2) < 0) goto osp_send_error;
    if (lio->write(lio, (uint8_t*)&TAIL, sizeof(HEADER)) < 0) goto osp_send_error;
    return 0;
osp_send_error:
    return -1;
}

static int m_read(io_t *io, void *buffer, size_t size)
{
    uint16_t length;
    uint16_t ck, ck_calc;
    uint16_t tail;
    osp_transport_t *ot = (osp_transport_t*)io;
    io_t *lio = ot->io;

    if (!scan_for_header(lio)) {
        if (read_exactly(lio, (uint8_t*)&length, 2) < 0) goto osp_recv_error;
        length = htobe16(length);
        if (length > size) {
            printf("osp_recv: recv buffer to small: %d < %d\n", size, length);
            return -1;
        }
        if (read_exactly(lio, buffer, length) < 0) goto osp_recv_error;
        if (read_exactly(lio, (uint8_t*)&ck, 2) < 0) goto osp_recv_error;
        if (read_exactly(lio, (uint8_t*)&tail, 2) < 0) goto osp_recv_error;
        ck = htobe16(ck);
        tail = htobe16(tail);
        if (tail != 0xb0b3) {
            printf("osp_recv: unexpected tail value: 0x%04x\n", tail);
            return -1;
        }
        ck_calc = osp_checksum(buffer, length);
        if(ck != ck_calc) {
            printf("osp_recv: invalid checksum. recv=0x%04x != calc=0x%04x\n", ck, ck_calc);
            return -1;
        }
        return length;
    }
osp_recv_error:
    return -1;
}

static int m_close(io_t *io)
{
    osp_transport_t *ot = (osp_transport_t*)io;
    return ot->io->close(ot->io);
}

io_t* osp_transport_alloc(io_t *io)
{
    osp_transport_t *ot = malloc(sizeof(osp_transport_t));
    ot->pub.open = m_open;
    ot->pub.write = m_write;
    ot->pub.read = m_read;
    ot->pub.close = m_close;
    ot->io = io;
    return (io_t*)ot;
}
