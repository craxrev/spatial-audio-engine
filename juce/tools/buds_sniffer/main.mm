// Galaxy Buds 2 Pro head-tracking sniffer.
// Throwaway diagnostic — connects to the buds via RFCOMM, sends the spatial-
// audio handshake, prints inbound BudGrv quaternion frames + stats.
//
// Build: ./build.sh
// Run:   ./buds_sniffer [seconds]                (default 30, free-form)
//        ./buds_sniffer protocol                 (guided calibration)
//        ./buds_sniffer probe <byte> [seconds]   (inject a CONTROL byte at midpoint;
//                                                 reports baseline vs post-probe rate)

#import <Foundation/Foundation.h>
#import <IOBluetooth/IOBluetooth.h>

#include <atomic>
#include <chrono>
#include <csignal>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <string>
#include <sys/select.h>
#include <unistd.h>
#include <vector>

// ---------- Protocol constants ----------
static const uint8_t SOM = 0xFD;
static const uint8_t EOM = 0xDD;

static const uint8_t MSG_SET_SPATIAL_AUDIO     = 124;
static const uint8_t MSG_SPATIAL_AUDIO_DATA    = 194;
static const uint8_t MSG_SPATIAL_AUDIO_CONTROL = 195;

static const uint8_t CTRL_ATTACH         = 0;
static const uint8_t CTRL_DETACH         = 1;
static const uint8_t CTRL_ATTACH_SUCCESS = 2;
static const uint8_t CTRL_DETACH_SUCCESS = 3;
static const uint8_t CTRL_KEEPALIVE      = 4;

static const uint8_t DATA_BUD_GRV       = 32;
static const uint8_t DATA_WEAR_ON       = 33;
static const uint8_t DATA_WEAR_OFF      = 34;
static const uint8_t DATA_BUD_GYROCAL   = 35;
static const uint8_t DATA_SENSOR_STUCK  = 36;

// Buds 2 Pro RFCOMM service UUID: 2e73a4ad-332d-41fc-90e2-16bef06523f2
static const uint8_t SPP_NEW_UUID[16] = {
    0x2e, 0x73, 0xa4, 0xad, 0x33, 0x2d, 0x41, 0xfc,
    0x90, 0xe2, 0x16, 0xbe, 0xf0, 0x65, 0x23, 0xf2
};

// ---------- CRC-16-CCITT (table from GalaxyBudsClient/Utils/Crc16.cs) ----------
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

static uint16_t crc16_ccitt(const uint8_t* data, size_t len) {
    uint16_t crc = 0;
    for (size_t i = 0; i < len; i++) {
        crc = CRC16_TAB[((crc >> 8) ^ data[i]) & 0xFF] ^ (uint16_t)(crc << 8);
    }
    return crc;
}

// ---------- Frame builder ----------
static std::vector<uint8_t> buildFrame(uint8_t msgId, const uint8_t* payload, size_t plen) {
    // size = MsgId(1) + payload + CRC(2)
    uint16_t size = (uint16_t)(plen + 3);
    // CRC over [MsgId, payload...]
    std::vector<uint8_t> crcData;
    crcData.reserve(plen + 1);
    crcData.push_back(msgId);
    for (size_t i = 0; i < plen; i++) crcData.push_back(payload[i]);
    uint16_t crc = crc16_ccitt(crcData.data(), crcData.size());

    std::vector<uint8_t> frame;
    frame.reserve(plen + 7);
    frame.push_back(SOM);
    frame.push_back((uint8_t)(size & 0xFF));        // header lo
    frame.push_back((uint8_t)((size >> 8) & 0xFF)); // header hi (no type/fragment bits for outbound Request)
    frame.push_back(msgId);
    for (size_t i = 0; i < plen; i++) frame.push_back(payload[i]);
    frame.push_back((uint8_t)(crc & 0xFF));         // crc lo
    frame.push_back((uint8_t)((crc >> 8) & 0xFF));  // crc hi
    frame.push_back(EOM);
    return frame;
}

// ---------- State ----------
static std::atomic<bool> g_quit{false};
static std::atomic<bool> g_attached{false};
static std::atomic<uint64_t> g_grvCount{0};
static std::atomic<int64_t>  g_firstGrvMs{-1};
static std::atomic<int64_t>  g_probeFiredMs{-1};
static std::atomic<uint64_t> g_grvCountPre{0};
static std::atomic<uint64_t> g_grvCountPost{0};
static std::atomic<int64_t>  g_firstPostMs{-1};
static std::atomic<int64_t>  g_lastGrvMs{-1};

static IOBluetoothRFCOMMChannel* g_channel = nil;
static std::vector<uint8_t> g_rxBuf;

static std::atomic<const char*> g_phase{"idle"};
static FILE* g_logFile = nullptr;

static void handleSigint(int) { g_quit = true; }

static int64_t nowMs() {
    using namespace std::chrono;
    return duration_cast<milliseconds>(steady_clock::now().time_since_epoch()).count();
}

static void send(uint8_t msgId, uint8_t payloadByte) {
    auto f = buildFrame(msgId, &payloadByte, 1);
    if (g_channel && [g_channel isOpen]) {
        [g_channel writeSync:(void*)f.data() length:(UInt16)f.size()];
    }
}

static void dumpHex(const uint8_t* p, size_t n) {
    for (size_t i = 0; i < n; i++) printf("%02X ", p[i]);
}

// ---------- Frame parser ----------
static void parseFrame(const uint8_t* frame, size_t total) {
    // frame: SOM | hdr_lo | hdr_hi | MsgId | payload[N] | crc_lo | crc_hi | EOM
    if (total < 7) return;
    uint16_t header = (uint16_t)frame[1] | ((uint16_t)frame[2] << 8);
    uint16_t size = header & 0x3FF;
    bool isFragment = (header & 0x2000) != 0;
    bool typeBit    = (header & 0x1000) != 0;
    uint8_t msgId = frame[3];
    int payloadLen = (int)size - 3;
    if (payloadLen < 0 || (size_t)(payloadLen + 7) > total) return;
    const uint8_t* payload = &frame[4];

    // CRC check over [MsgId, payload...]
    std::vector<uint8_t> crcData;
    crcData.reserve(payloadLen + 1);
    crcData.push_back(msgId);
    for (int i = 0; i < payloadLen; i++) crcData.push_back(payload[i]);
    uint16_t crcCalc = crc16_ccitt(crcData.data(), crcData.size());
    uint16_t crcRecv = (uint16_t)payload[payloadLen] | ((uint16_t)payload[payloadLen + 1] << 8);
    if (crcCalc != crcRecv) {
        printf("[crc-fail] msgId=%u size=%u  calc=%04X recv=%04X\n", msgId, size, crcCalc, crcRecv);
        return;
    }

    switch (msgId) {
        case MSG_SPATIAL_AUDIO_CONTROL: {
            uint8_t result = payloadLen > 0 ? payload[0] : 0xFF;
            const char* name = "?";
            if (result == CTRL_ATTACH_SUCCESS) { name = "AttachSuccess"; g_attached = true; }
            else if (result == CTRL_DETACH_SUCCESS) { name = "DetachSuccess"; g_attached = false; }
            printf("[ctrl] %s (%u)  hdrBits[type=%d frag=%d]  payloadLen=%d",
                   name, result, typeBit, isFragment, payloadLen);
            if (payloadLen > 1) { printf("  extra="); dumpHex(&payload[1], payloadLen - 1); }
            printf("\n");
            break;
        }
        case MSG_SPATIAL_AUDIO_DATA: {
            if (payloadLen < 1) return;
            uint8_t evt = payload[0];
            const uint8_t* d = &payload[1];
            int dlen = payloadLen - 1;
            switch (evt) {
                case DATA_BUD_GRV: {
                    if (dlen < 9) { printf("[grv] short(%d)\n", dlen); return; }
                    int16_t raw[4];
                    for (int i = 0; i < 4; i++) {
                        raw[i] = (int16_t)((uint16_t)d[i*2] | ((uint16_t)d[i*2+1] << 8));
                    }
                    float q[4] = {raw[0]/10000.0f, raw[1]/10000.0f, raw[2]/10000.0f, raw[3]/10000.0f};
                    uint8_t valid = d[8];
                    uint64_t n = ++g_grvCount;
                    int64_t t = nowMs();
                    int64_t first = g_firstGrvMs.load();
                    if (first < 0) g_firstGrvMs = t;
                    g_lastGrvMs = t;
                    int64_t fired = g_probeFiredMs.load();
                    if (fired < 0) {
                        ++g_grvCountPre;
                    } else {
                        if (g_firstPostMs.load() < 0) g_firstPostMs = t;
                        ++g_grvCountPost;
                    }
                    if (g_logFile) {
                        fprintf(g_logFile, "GRV %s %lld %d %d %d %d %u\n",
                                g_phase.load(), (long long)t, raw[0], raw[1], raw[2], raw[3], valid);
                    } else if (n <= 5 || n % 25 == 0) {
                        // Free-form mode: thin output to terminal.
                        printf("[grv #%llu @%lldms] raw=[%6d %6d %6d %6d] -> q=[%+.4f %+.4f %+.4f %+.4f] valid=%u\n",
                               (unsigned long long)n, (long long)t, raw[0], raw[1], raw[2], raw[3],
                               q[0], q[1], q[2], q[3], valid);
                    }
                    break;
                }
                case DATA_WEAR_ON:  printf("[evt] WearOn\n"); break;
                case DATA_WEAR_OFF: printf("[evt] WearOff\n"); break;
                case DATA_BUD_GYROCAL: printf("[evt] BudGyrocal (len=%d)\n", dlen); break;
                case DATA_SENSOR_STUCK: printf("[evt] SensorStuck\n"); break;
                default:
                    printf("[data] evt=%u dlen=%d bytes=", evt, dlen);
                    dumpHex(d, dlen); printf("\n");
                    break;
            }
            break;
        }
        default:
            printf("[msg %u] size=%u payload=", msgId, size);
            dumpHex(payload, payloadLen); printf("\n");
            break;
    }
}

// Re-frame the byte stream. State machine: scan for SOM, read header, read body, verify EOM.
static void onIncomingBytes(const uint8_t* bytes, size_t n) {
    g_rxBuf.insert(g_rxBuf.end(), bytes, bytes + n);

    for (;;) {
        // Find SOM
        size_t som = SIZE_MAX;
        for (size_t i = 0; i < g_rxBuf.size(); i++) {
            if (g_rxBuf[i] == SOM) { som = i; break; }
        }
        if (som == SIZE_MAX) { g_rxBuf.clear(); return; }
        if (som > 0) g_rxBuf.erase(g_rxBuf.begin(), g_rxBuf.begin() + som);

        if (g_rxBuf.size() < 7) return; // need at least SOM+hdr+msgId+crc+EOM (worst case empty payload)
        uint16_t header = (uint16_t)g_rxBuf[1] | ((uint16_t)g_rxBuf[2] << 8);
        uint16_t size = header & 0x3FF;
        size_t total = 1 + 2 + size + 1; // SOM + hdr + (MsgId+payload+CRC) + EOM
        if (g_rxBuf.size() < total) return;
        if (g_rxBuf[total - 1] != EOM) {
            // Drop the SOM and resync.
            g_rxBuf.erase(g_rxBuf.begin());
            continue;
        }
        parseFrame(g_rxBuf.data(), total);
        g_rxBuf.erase(g_rxBuf.begin(), g_rxBuf.begin() + total);
    }
}

// ---------- Pump / wait helpers ----------
static int64_t g_lastKA = 0;

static void pumpOnce() {
    [[NSRunLoop currentRunLoop] runUntilDate:[NSDate dateWithTimeIntervalSinceNow:0.05]];
    int64_t now = nowMs();
    if (now - g_lastKA >= 2000) {
        send(MSG_SPATIAL_AUDIO_CONTROL, CTRL_KEEPALIVE);
        g_lastKA = now;
    }
}

static bool stdinHasInput() {
    fd_set fds; FD_ZERO(&fds); FD_SET(STDIN_FILENO, &fds);
    timeval tv = {0, 0};
    return select(STDIN_FILENO + 1, &fds, nullptr, nullptr, &tv) > 0;
}

static void waitForEnter() {
    while (!g_quit.load()) {
        pumpOnce();
        if (stdinHasInput()) {
            char buf[256];
            if (fgets(buf, sizeof(buf), stdin) == nullptr) {}
            return;
        }
    }
}

// ---------- Protocol driver ----------
struct Phase { const char* tag; const char* prompt; };
static const Phase kPhases[] = {
    {"rest_1",     "LOOK STRAIGHT AHEAD. Head level, eyes on something in front of you. Don't move."},
    {"yaw_left",   "Without tilting, TURN YOUR HEAD TO THE LEFT (like saying NO, but stop and hold). Your nose points to your left."},
    {"rest_2",     "Look STRAIGHT AHEAD again."},
    {"yaw_right",  "TURN YOUR HEAD TO THE RIGHT. Your nose points to your right."},
    {"rest_3",     "Look STRAIGHT AHEAD again."},
    {"pitch_down", "LOOK DOWN — point your chin toward your chest. (Like nodding yes, but stop at the bottom.)"},
    {"rest_4",     "Look STRAIGHT AHEAD again."},
    {"pitch_up",   "LOOK UP — point your chin toward the ceiling."},
    {"rest_5",     "Look STRAIGHT AHEAD again."},
    {"roll_left",  "TILT YOUR HEAD: drop your LEFT EAR toward your LEFT SHOULDER. Keep facing forward (don't turn)."},
    {"rest_6",     "Look STRAIGHT AHEAD again."},
    {"roll_right", "TILT YOUR HEAD: drop your RIGHT EAR toward your RIGHT SHOULDER. Keep facing forward."},
};

static void runProtocol() {
    const int holdSec = 6;
    int n = (int)(sizeof(kPhases) / sizeof(kPhases[0]));

    g_logFile = fopen("protocol_log.txt", "w");
    if (!g_logFile) { fprintf(stderr, "Could not open protocol_log.txt\n"); return; }
    fprintf(g_logFile, "# Galaxy Buds 2 Pro head-tracking protocol log\n");
    fprintf(g_logFile, "# Format: GRV <phase> <ms> <raw0> <raw1> <raw2> <raw3> <valid>\n");
    fprintf(g_logFile, "# Phase markers: PHASE_BEGIN/PHASE_END <tag> <ms>\n");

    printf("\n=== PROTOCOL MODE ===\n");
    printf("%d phases, %d s each. Press ENTER between phases to begin recording.\n", n, holdSec);
    printf("Only the middle 3 s of each phase is analyzed, so don't sweat timing.\n");
    printf("Stay still during the countdown. Beep = recording start / end.\n\n");

    for (int i = 0; i < n && !g_quit.load(); i++) {
        printf("[%d/%d] %s\n", i + 1, n, kPhases[i].prompt);
        printf("       Get into position. Press ENTER to start recording... ");
        fflush(stdout);
        waitForEnter();
        if (g_quit.load()) break;

        g_phase = kPhases[i].tag;
        int64_t t0 = nowMs();
        fprintf(g_logFile, "PHASE_BEGIN %s %lld\n", kPhases[i].tag, (long long)t0);
        fflush(g_logFile);
        printf("       \aRECORDING — hold still ...\n");

        int64_t start = nowMs();
        while (!g_quit.load() && (nowMs() - start) < holdSec * 1000) {
            pumpOnce();
            int remaining = (int)((holdSec * 1000 - (nowMs() - start) + 999) / 1000);
            printf("\r       %d ...   ", remaining);
            fflush(stdout);
        }

        int64_t t1 = nowMs();
        fprintf(g_logFile, "PHASE_END %s %lld\n", kPhases[i].tag, (long long)t1);
        fflush(g_logFile);
        g_phase = "idle";
        printf("\r       done.    \a\n\n");
    }

    fclose(g_logFile);
    g_logFile = nullptr;
    printf("=== PROTOCOL COMPLETE ===\n");
    printf("Log written: %s/protocol_log.txt\n",
           [[[NSFileManager defaultManager] currentDirectoryPath] UTF8String]);
}

// ---------- RFCOMM delegate ----------
@interface BudsDelegate : NSObject <IOBluetoothRFCOMMChannelDelegate, IOBluetoothDeviceAsyncCallbacks> {
@public
    BOOL sdpDone;
}
@end

@implementation BudsDelegate
- (id)init { if ((self = [super init])) { sdpDone = NO; } return self; }
- (void)rfcommChannelData:(IOBluetoothRFCOMMChannel*)ch data:(void*)data length:(size_t)len {
    onIncomingBytes((const uint8_t*)data, len);
}
- (void)rfcommChannelClosed:(IOBluetoothRFCOMMChannel*)ch {
    printf("[rfcomm] channel closed\n");
    g_quit = true;
}
- (void)rfcommChannelOpenComplete:(IOBluetoothRFCOMMChannel*)ch status:(IOReturn)status {}
- (void)sdpQueryComplete:(IOBluetoothDevice*)device status:(IOReturn)status { sdpDone = YES; }
- (void)remoteNameRequestComplete:(IOBluetoothDevice*)device status:(IOReturn)status {}
- (void)connectionComplete:(IOBluetoothDevice*)device status:(IOReturn)status {}
@end

// ---------- Device discovery ----------
static IOBluetoothDevice* findBudsDevice() {
    NSArray* paired = [IOBluetoothDevice pairedDevices];
    NSLog(@"Scanning %lu paired devices...", (unsigned long)[paired count]);
    for (IOBluetoothDevice* dev in paired) {
        NSString* name = dev.name ?: @"(no name)";
        NSLog(@"  - %@  [%@]", name, dev.addressString);
        if ([name rangeOfString:@"Buds" options:NSCaseInsensitiveSearch].location != NSNotFound) {
            NSLog(@"  -> match");
            return dev;
        }
    }
    return nil;
}

// ---------- Main ----------
int main(int argc, char** argv) {
    @autoreleasepool {
        bool protocolMode = (argc >= 2 && strcmp(argv[1], "protocol") == 0);
        int probeByte = -1;          // -1 = no probe
        int seconds = 30;
        if (!protocolMode) {
            // Parse:  [seconds]  |  probe <byte> [seconds]
            int i = 1;
            if (i < argc && strcmp(argv[i], "probe") == 0) {
                if (i + 1 >= argc) { fprintf(stderr, "usage: buds_sniffer probe <byte> [seconds]\n"); return 1; }
                probeByte = atoi(argv[i + 1]);
                if (probeByte < 0 || probeByte > 255) { fprintf(stderr, "probe byte must be 0..255\n"); return 1; }
                i += 2;
            }
            if (i < argc) {
                seconds = atoi(argv[i]);
                if (seconds <= 0) seconds = 30;
            }
        }
        if (protocolMode) {
            printf("buds_sniffer — protocol mode (guided). Ctrl-C to abort.\n");
        } else if (probeByte >= 0) {
            printf("buds_sniffer — probe byte %d, duration %d s "
                   "(baseline %ds + post-probe %ds). Ctrl-C to stop early.\n",
                   probeByte, seconds, seconds / 2, seconds - seconds / 2);
        } else {
            printf("buds_sniffer — duration %d s. Ctrl-C to stop early. "
                   "(run with 'protocol' or 'probe <byte>')\n", seconds);
        }

        signal(SIGINT, handleSigint);

        IOBluetoothDevice* device = findBudsDevice();
        if (!device) { fprintf(stderr, "No Buds device found in paired list.\n"); return 1; }

        if (![device isConnected]) {
            IOReturn r = [device openConnection];
            if (r != kIOReturnSuccess) {
                fprintf(stderr, "openConnection failed: %d\n", r);
                return 2;
            }
        }

        BudsDelegate* delegate = [[BudsDelegate alloc] init];

        IOBluetoothSDPUUID* uuid = [IOBluetoothSDPUUID uuidWithBytes:SPP_NEW_UUID length:16];
        [device performSDPQuery:delegate];
        for (int i = 0; i < 30 && !delegate->sdpDone; i++) {
            [NSThread sleepForTimeInterval:0.1];
        }

        IOBluetoothSDPServiceRecord* svc = [device getServiceRecordForUUID:uuid];
        if (!svc) { fprintf(stderr, "Service record for SppNew not found on device.\n"); return 3; }

        UInt8 channelID = 0;
        if ([svc getRFCOMMChannelID:&channelID] != kIOReturnSuccess) {
            fprintf(stderr, "getRFCOMMChannelID failed\n"); return 4;
        }
        printf("[rfcomm] '%s' channel=%u\n", [[svc getServiceName] UTF8String] ?: "?", channelID);

        IOBluetoothRFCOMMChannel* ch = nil;
        IOReturn or_ = [device openRFCOMMChannelSync:&ch withChannelID:channelID delegate:delegate];
        (void)or_;
        // Wait for channel to actually open (per macOS quirk in GalaxyBudsClient).
        for (int i = 0; i < 30 && ![ch isOpen]; i++) [NSThread sleepForTimeInterval:0.1];
        if (![ch isOpen]) { fprintf(stderr, "RFCOMM did not open\n"); return 5; }
        g_channel = ch;
        printf("[rfcomm] open\n");

        // Handshake.
        printf("[tx] SET_SPATIAL_AUDIO 1\n");
        send(MSG_SET_SPATIAL_AUDIO, 0x01);
        [NSThread sleepForTimeInterval:0.2];

        printf("[tx] CONTROL Attach\n");
        send(MSG_SPATIAL_AUDIO_CONTROL, CTRL_ATTACH);

        // Wait briefly for AttachSuccess to land (some firmwares skip the ack — don't block).
        g_lastKA = nowMs();
        for (int i = 0; i < 20 && !g_attached.load(); i++) [NSThread sleepForTimeInterval:0.05];

        if (protocolMode) {
            runProtocol();
        } else if (probeByte >= 0) {
            int64_t start = nowMs();
            int64_t mid = start + (int64_t)(seconds / 2) * 1000;
            int64_t end = start + (int64_t)seconds * 1000;
            bool probed = false;
            while (!g_quit.load() && nowMs() < end) {
                pumpOnce();
                if (!probed && nowMs() >= mid) {
                    printf("\n[probe] sending CONTROL byte %d  (t=%lldms)\n",
                           probeByte, (long long)(nowMs() - start));
                    send(MSG_SPATIAL_AUDIO_CONTROL, (uint8_t)probeByte);
                    g_probeFiredMs = nowMs();
                    probed = true;
                }
            }
        } else {
            int64_t start = nowMs();
            while (!g_quit.load() && (nowMs() - start) < (int64_t)seconds * 1000) {
                pumpOnce();
            }
        }

        // Teardown.
        printf("[tx] CONTROL Detach\n");
        send(MSG_SPATIAL_AUDIO_CONTROL, CTRL_DETACH);
        [NSThread sleepForTimeInterval:0.2];
        printf("[tx] SET_SPATIAL_AUDIO 0\n");
        send(MSG_SET_SPATIAL_AUDIO, 0x00);
        [NSThread sleepForTimeInterval:0.2];

        [g_channel closeChannel];
        g_channel = nil;

        // Stats.
        uint64_t n = g_grvCount.load();
        int64_t first = g_firstGrvMs.load();
        int64_t lastGrv = g_lastGrvMs.load();
        double rate = 0.0;
        if (n > 1 && first > 0 && lastGrv > first) {
            rate = (double)(n - 1) * 1000.0 / (double)(lastGrv - first);
        }
        printf("\n[summary] BudGrv frames: %llu  est-rate: %.1f Hz\n",
               (unsigned long long)n, rate);

        int64_t fired = g_probeFiredMs.load();
        if (fired > 0) {
            uint64_t nPre  = g_grvCountPre.load();
            uint64_t nPost = g_grvCountPost.load();
            int64_t fPost  = g_firstPostMs.load();
            double rPre = 0.0, rPost = 0.0;
            if (nPre > 1 && first > 0 && fired > first) {
                rPre = (double)(nPre - 1) * 1000.0 / (double)(fired - first);
            }
            if (nPost > 1 && fPost > 0 && lastGrv > fPost) {
                rPost = (double)(nPost - 1) * 1000.0 / (double)(lastGrv - fPost);
            }
            printf("[probe]   baseline: %llu frames  %.1f Hz   "
                   "post-probe: %llu frames  %.1f Hz   "
                   "delta: %+.1f Hz\n",
                   (unsigned long long)nPre, rPre,
                   (unsigned long long)nPost, rPost,
                   rPost - rPre);
        }
    }
    return 0;
}
