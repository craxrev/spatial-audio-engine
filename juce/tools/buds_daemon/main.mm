// buds_daemon — bridges Galaxy Buds 2 Pro IMU into a UDP/OSC stream so an
// AU plugin (which can't access Bluetooth inside its host) can consume it.
//
// Sends `/headpose ,ffff w x y z` to 127.0.0.1:9000 at ~30 Hz.
// Protocol details: juce/docs/headtracking_protocol.md
//
// Build: ./build.sh
// Run:   ./buds_daemon                (default port 9000)
//        ./buds_daemon --port 9001

#import <Foundation/Foundation.h>
#import <IOBluetooth/IOBluetooth.h>

#include <arpa/inet.h>
#include <atomic>
#include <chrono>
#include <csignal>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <netinet/in.h>
#include <string>
#include <sys/socket.h>
#include <thread>
#include <unistd.h>
#include <vector>

// ---------- Protocol constants ----------
static const uint8_t SOM = 0xFD;
static const uint8_t EOM = 0xDD;
static const uint8_t MSG_SET_SPATIAL_AUDIO     = 124;
static const uint8_t MSG_SPATIAL_AUDIO_DATA    = 194;
static const uint8_t MSG_SPATIAL_AUDIO_CONTROL = 195;
static const uint8_t CTRL_ATTACH    = 0;
static const uint8_t CTRL_DETACH    = 1;
static const uint8_t CTRL_KEEPALIVE = 4;
static const uint8_t DATA_BUD_GRV   = 32;

// SppNew UUID for Buds 2 Pro.
static const uint8_t SPP_NEW_UUID[16] = {
    0x2e, 0x73, 0xa4, 0xad, 0x33, 0x2d, 0x41, 0xfc,
    0x90, 0xe2, 0x16, 0xbe, 0xf0, 0x65, 0x23, 0xf2
};

static const uint16_t CRC16_TAB[256] = {
    0,4129,8258,12387,16516,20645,24774,28903,33032,37161,41290,
    45419,49548,53677,57806,61935,4657,528,12915,8786,21173,17044,29431,25302,
    37689,33560,45947,41818,54205,50076,62463,58334,9314,13379,1056,5121,25830,
    29895,17572,21637,42346,46411,34088,38153,58862,62927,50604,54669,13907,
    9842,5649,1584,30423,26358,22165,18100,46939,42874,38681,34616,63455,59390,
    55197,51132,18628,22757,26758,30887,2112,6241,10242,14371,51660,55789,
    59790,63919,35144,39273,43274,47403,23285,19156,31415,27286,6769,2640,
    14899,10770,56317,52188,64447,60318,39801,35672,47931,43802,27814,31879,
    19684,23749,11298,15363,3168,7233,60846,64911,52716,56781,44330,48395,
    36200,40265,32407,28342,24277,20212,15891,11826,7761,3696,65439,61374,
    57309,53244,48923,44858,40793,36728,37256,33193,45514,41451,53516,49453,
    61774,57711,4224,161,12482,8419,20484,16421,28742,24679,33721,37784,41979,
    46042,49981,54044,58239,62302,689,4752,8947,13010,16949,21012,25207,29270,
    46570,42443,38312,34185,62830,58703,54572,50445,13538,9411,5280,1153,29798,
    25671,21540,17413,42971,47098,34713,38840,59231,63358,50973,55100,9939,
    14066,1681,5808,26199,30326,17941,22068,55628,51565,63758,59695,39368,
    35305,47498,43435,22596,18533,30726,26663,6336,2273,14466,10403,52093,
    56156,60223,64286,35833,39896,43963,48026,19061,23124,27191,31254,2801,
    6864,10931,14994,64814,60687,56684,52557,48554,44427,40424,36297,31782,
    27655,23652,19525,15522,11395,7392,3265,61215,65342,53085,57212,44955,
    49082,36825,40952,28183,32310,20053,24180,11923,16050,3793,7920
};

static uint16_t crc16Ccitt(const uint8_t* data, size_t len) {
    uint16_t crc = 0;
    for (size_t i = 0; i < len; i++)
        crc = CRC16_TAB[((crc >> 8) ^ data[i]) & 0xFF] ^ (uint16_t)(crc << 8);
    return crc;
}

static std::vector<uint8_t> buildFrame(uint8_t msgId, uint8_t b) {
    uint8_t in[2] = {msgId, b};
    uint16_t crc = crc16Ccitt(in, 2);
    return {SOM, 0x04, 0x00, msgId, b, (uint8_t)(crc & 0xFF), (uint8_t)((crc >> 8) & 0xFF), EOM};
}

// ---------- OSC (hand-encoded, no library) ----------
// Packet: address "/headpose"\0 padded to 12 bytes, type tag ",ffff"\0 padded
// to 8 bytes, 4 × float32 big-endian.
static int g_udpFd = -1;
static struct sockaddr_in g_udpDst;

static void openUdp(int port) {
    g_udpFd = socket(AF_INET, SOCK_DGRAM, 0);
    if (g_udpFd < 0) { perror("socket"); std::exit(2); }
    std::memset(&g_udpDst, 0, sizeof(g_udpDst));
    g_udpDst.sin_family = AF_INET;
    g_udpDst.sin_port   = htons((uint16_t)port);
    inet_pton(AF_INET, "127.0.0.1", &g_udpDst.sin_addr);
}

static uint32_t floatToNetBytes(float f) {
    uint32_t u;
    std::memcpy(&u, &f, 4);
    return htonl(u);
}

static void sendHeadpose(float w, float x, float y, float z) {
    uint8_t pkt[36];
    std::memset(pkt, 0, sizeof(pkt));
    std::memcpy(pkt + 0, "/headpose", 9);   // 9 chars, +3 padding zero bytes
    std::memcpy(pkt + 12, ",ffff", 5);      // 5 chars, +3 padding zero bytes
    uint32_t f0 = floatToNetBytes(w);
    uint32_t f1 = floatToNetBytes(x);
    uint32_t f2 = floatToNetBytes(y);
    uint32_t f3 = floatToNetBytes(z);
    std::memcpy(pkt + 20, &f0, 4);
    std::memcpy(pkt + 24, &f1, 4);
    std::memcpy(pkt + 28, &f2, 4);
    std::memcpy(pkt + 32, &f3, 4);
    (void)sendto(g_udpFd, pkt, sizeof(pkt), 0,
                 (struct sockaddr*)&g_udpDst, sizeof(g_udpDst));
}

// ---------- BT state ----------
static std::atomic<bool> g_quit{false};
static IOBluetoothRFCOMMChannel* g_channel = nil;
static std::vector<uint8_t> g_rxBuf;
static std::atomic<uint64_t> g_grvCount{0};

static void handleSigint(int) { g_quit = true; }

static void parseFrame(const uint8_t* frame, size_t total) {
    if (total < 7) return;
    uint16_t header = (uint16_t)frame[1] | ((uint16_t)frame[2] << 8);
    uint16_t size = header & 0x3FF;
    uint8_t msgId = frame[3];
    int payloadLen = (int)size - 3;
    if (payloadLen < 0 || (size_t)(payloadLen + 7) > total) return;
    const uint8_t* payload = &frame[4];

    std::vector<uint8_t> crcData;
    crcData.reserve(payloadLen + 1);
    crcData.push_back(msgId);
    for (int i = 0; i < payloadLen; i++) crcData.push_back(payload[i]);
    uint16_t crcCalc = crc16Ccitt(crcData.data(), crcData.size());
    uint16_t crcRecv = (uint16_t)payload[payloadLen] | ((uint16_t)payload[payloadLen + 1] << 8);
    if (crcCalc != crcRecv) return;

    if (msgId != MSG_SPATIAL_AUDIO_DATA || payloadLen < 1) return;
    if (payload[0] != DATA_BUD_GRV) return;
    if (payloadLen < 9) return;  // 4×int16 (8 bytes) + flag byte
    const uint8_t* d = &payload[1];
    int16_t raw[4];
    for (int i = 0; i < 4; i++)
        raw[i] = (int16_t)((uint16_t)d[i*2] | ((uint16_t)d[i*2+1] << 8));
    // d[8] is some flag the buds send (counter/state — meaning unclear). The
    // GalaxyBudsClient C# reference stores it as `GrvBoolean = (d[8] == 0)`
    // but never gates the quaternion on it. Don't filter.
    // Wire order is WXYZ per empirical analysis.
    const float w = raw[0] / 10000.0f;
    const float x = raw[1] / 10000.0f;
    const float y = raw[2] / 10000.0f;
    const float z = raw[3] / 10000.0f;
    sendHeadpose(w, x, y, z);
    ++g_grvCount;
}

static void onIncomingBytes(const uint8_t* bytes, size_t n) {
    g_rxBuf.insert(g_rxBuf.end(), bytes, bytes + n);
    for (;;) {
        size_t som = SIZE_MAX;
        for (size_t i = 0; i < g_rxBuf.size(); i++)
            if (g_rxBuf[i] == SOM) { som = i; break; }
        if (som == SIZE_MAX) { g_rxBuf.clear(); return; }
        if (som > 0) g_rxBuf.erase(g_rxBuf.begin(), g_rxBuf.begin() + som);
        if (g_rxBuf.size() < 7) return;
        uint16_t header = (uint16_t)g_rxBuf[1] | ((uint16_t)g_rxBuf[2] << 8);
        uint16_t size = header & 0x3FF;
        size_t total = 1 + 2 + size + 1;
        if (g_rxBuf.size() < total) return;
        if (g_rxBuf[total - 1] != EOM) { g_rxBuf.erase(g_rxBuf.begin()); continue; }
        parseFrame(g_rxBuf.data(), total);
        g_rxBuf.erase(g_rxBuf.begin(), g_rxBuf.begin() + total);
    }
}

@interface BudsDelegate : NSObject <IOBluetoothRFCOMMChannelDelegate,
                                    IOBluetoothDeviceAsyncCallbacks>
@property (atomic) BOOL sdpDone;
@property (atomic) BOOL channelClosed;
@end
@implementation BudsDelegate
- (instancetype)init { if ((self = [super init])) { self.sdpDone = NO; self.channelClosed = NO; } return self; }
- (void)rfcommChannelData:(IOBluetoothRFCOMMChannel*)ch data:(void*)data length:(size_t)len {
    onIncomingBytes((const uint8_t*)data, len);
}
- (void)rfcommChannelClosed:(IOBluetoothRFCOMMChannel*)ch { self.channelClosed = YES; }
- (void)rfcommChannelOpenComplete:(IOBluetoothRFCOMMChannel*)ch status:(IOReturn)st {}
- (void)sdpQueryComplete:(IOBluetoothDevice*)d status:(IOReturn)st { self.sdpDone = YES; }
- (void)remoteNameRequestComplete:(IOBluetoothDevice*)d status:(IOReturn)st {}
- (void)connectionComplete:(IOBluetoothDevice*)d status:(IOReturn)st {}
@end

static void sendCtrl(uint8_t msgId, uint8_t b) {
    auto f = buildFrame(msgId, b);
    if (g_channel && [g_channel isOpen])
        [g_channel writeSync:(void*)f.data() length:(UInt16)f.size()];
}

int main(int argc, char** argv) {
    int port = 9000;
    for (int i = 1; i < argc; ++i) {
        if (std::strcmp(argv[i], "--port") == 0 && i + 1 < argc)
            port = std::atoi(argv[++i]);
    }
    signal(SIGINT, handleSigint);
    signal(SIGTERM, handleSigint);
    openUdp(port);
    printf("buds_daemon — streaming /headpose to udp://127.0.0.1:%d\n", port);
    printf("Ctrl-C to quit. Quit GalaxyBudsClient first (RFCOMM is exclusive).\n\n");

    @autoreleasepool {
        while (!g_quit.load()) {
            @autoreleasepool {
                IOBluetoothDevice* device = nil;
                for (IOBluetoothDevice* d in [IOBluetoothDevice pairedDevices]) {
                    if (d.name && [d.name rangeOfString:@"Buds"
                                                options:NSCaseInsensitiveSearch].location != NSNotFound)
                    { device = d; break; }
                }
                if (!device) {
                    printf("[daemon] no paired Galaxy Buds found; retrying in 5s\n");
                    for (int i = 0; i < 100 && !g_quit.load(); ++i)
                        std::this_thread::sleep_for(std::chrono::milliseconds(50));
                    continue;
                }
                printf("[daemon] found %s\n", [device.name UTF8String]);

                // Force an explicit openConnection regardless of isConnected.
                // When the buds are idle (no audio), isConnected can be true
                // while the link is too suspended to accept new RFCOMM.
                {
                    IOReturn rr = [device openConnection];
                    printf("[daemon] openConnection -> %d (isConnected=%d)\n",
                           (int)rr, (int)[device isConnected]);
                    if (rr != kIOReturnSuccess && ![device isConnected]) {
                        printf("[daemon] cannot open base connection; retry in 2s\n");
                        for (int i = 0; i < 40 && !g_quit.load(); ++i)
                            std::this_thread::sleep_for(std::chrono::milliseconds(50));
                        continue;
                    }
                }

                BudsDelegate* delegate = [[BudsDelegate alloc] init];
                IOBluetoothSDPUUID* uuid = [IOBluetoothSDPUUID uuidWithBytes:SPP_NEW_UUID length:16];
                [device performSDPQuery:delegate];
                for (int i = 0; i < 30 && !delegate.sdpDone && !g_quit.load(); ++i)
                    std::this_thread::sleep_for(std::chrono::milliseconds(50));
                printf("[daemon] sdpDone=%d\n", (int)delegate.sdpDone);

                IOBluetoothSDPServiceRecord* svc = [device getServiceRecordForUUID:uuid];
                UInt8 chId = 0;
                if (!svc || [svc getRFCOMMChannelID:&chId] != kIOReturnSuccess) {
                    printf("[daemon] SppNew service not found; retry in 3s\n");
                    for (int i = 0; i < 60 && !g_quit.load(); ++i)
                        std::this_thread::sleep_for(std::chrono::milliseconds(50));
                    continue;
                }
                printf("[daemon] '%s' chan=%u\n",
                       [[svc getServiceName] UTF8String] ?: "?", chId);

                IOBluetoothRFCOMMChannel* channel = nil;
                IOReturn orr = [device openRFCOMMChannelSync:&channel
                                              withChannelID:chId
                                                   delegate:delegate];
                // Per GalaxyBudsClient's mac port: this commonly returns
                // kIOReturnError (-536870212) but the channel still opens
                // asynchronously via the delegate. We pump the runloop and
                // wait up to 5 s for isOpen to flip.
                printf("[daemon] openRFCOMMChannelSync -> %d, channel=%s (waiting for isOpen)\n",
                       (int)orr, channel == nil ? "nil" : "non-nil");
                for (int i = 0; i < 100 && (channel == nil || ![channel isOpen]) && !g_quit.load(); ++i)
                {
                    [[NSRunLoop currentRunLoop] runUntilDate:[NSDate dateWithTimeIntervalSinceNow:0.05]];
                    std::this_thread::sleep_for(std::chrono::milliseconds(10));
                }
                if (channel == nil || ![channel isOpen]) {
                    printf("[daemon] RFCOMM did not open after 5s. Try playing audio on "
                           "the buds briefly to wake the radio, then re-run.\n");
                    for (int i = 0; i < 60 && !g_quit.load(); ++i)
                        std::this_thread::sleep_for(std::chrono::milliseconds(50));
                    continue;
                }
                g_channel = channel;

                sendCtrl(MSG_SET_SPATIAL_AUDIO,     0x01);
                sendCtrl(MSG_SPATIAL_AUDIO_CONTROL, CTRL_ATTACH);
                printf("[daemon] streaming (chan=%u)\n", chId);

                auto lastKA = std::chrono::steady_clock::now();
                auto lastStat = lastKA;
                uint64_t lastCount = 0;
                while (!g_quit.load() && !delegate.channelClosed) {
                    auto t0 = std::chrono::steady_clock::now();
                    [[NSRunLoop currentRunLoop] runUntilDate:[NSDate dateWithTimeIntervalSinceNow:0.02]];

                    auto now = std::chrono::steady_clock::now();
                    if (now - lastKA >= std::chrono::seconds(2)) {
                        sendCtrl(MSG_SPATIAL_AUDIO_CONTROL, CTRL_KEEPALIVE);
                        lastKA = now;
                    }
                    if (now - lastStat >= std::chrono::seconds(5)) {
                        uint64_t c = g_grvCount.load();
                        double rate = (double)(c - lastCount) / 5.0;
                        printf("[daemon] %.1f Hz (%llu frames total)\n", rate, (unsigned long long)c);
                        lastCount = c;
                        lastStat = now;
                    }

                    auto el = std::chrono::steady_clock::now() - t0;
                    if (el < std::chrono::milliseconds(20))
                        std::this_thread::sleep_for(std::chrono::milliseconds(20) - el);
                }

                sendCtrl(MSG_SPATIAL_AUDIO_CONTROL, CTRL_DETACH);
                sendCtrl(MSG_SET_SPATIAL_AUDIO,     0x00);
                [channel closeChannel];
                g_channel = nil;
                printf("[daemon] disconnected; will retry\n");
            }
        }
    }
    if (g_udpFd >= 0) close(g_udpFd);
    return 0;
}
