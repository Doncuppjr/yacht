package trainer

// Bluetooth (FrSKY's PARA trainer protocol) link

import (
	"sync/atomic"
	"time"

	"tinygo.org/x/bluetooth"
)

// 'C' is for Commands (runtime effects)
const (
	CHAR_COMMANDS         = 0xFFC1
	CHAR_COMMANDS_COMPAT  = 0xAFF2 // Cliff's Head Tracker reset command characteristic
	CMD_ORIENTATION_RESET = 'R'    // reset orientation
	CMD_FACTORY_RESET     = 'F'    // factory reset
	CMD_REBOOT            = 'B'    // reboot device
)

// 'D' is for Data (to persist)
const (
	// device name (up to 16 bytes) - padded with zeros
	CHAR_DATA_DEVICE_NAME = 0xFFD1

	// axis mapping (3 bytes) - one byte per axis
	//
	// each byte has format: 0b00IE0OOO where
	// - '0'   bit is not used,
	// - 'I'   bit for inverted(1)/not inverted(0),
	// - 'E'   bit for enabled(1)/disabled(0)
	// - 'OOO' three bits for channel index offset (0-7),
	//
	// examples:
	// - 0x10 means axis mapped to channel 1 (offset 0), enabled,  not inverted
	// - 0x11 means axis mapped to channel 2 (offset 1), enabled,  not inverted
	// - 0x25 means axis mapped to channel 6 (offset 5), disabled, inverted
	// - 0x34 means axis mapped to channel 5 (offset 4), enabled,  inverted
	//
	// default mapping value: "0x101112" or "16 17 18" (first 3 channels, enabled, not inverted)
	CHAR_DATA_AXIS_MAPPING = 0xFFD2
)

// Have to send this to master radio on connect otherwise high chance opentx para code will never receive "Connected" message
// Since it looks for "Connected\r\n" and sometimes(?) bluetooth underlying layer on master radio
// never sends "\r\n" and starts sending trainer data directly
var bootBuffer = []byte{0x0d, 0x0a}

type Para struct {
	adapter    *bluetooth.Adapter
	adv        *bluetooth.Advertisement
	fff6Handle bluetooth.Characteristic

	callbackHandler CallbackHandler

	buffer    [20]byte
	sendAfter time.Time

	paired bool
	// NRF SoftDevice cannot evict queued notifications. When its queue is
	// full, skip obsolete position samples and only submit fresh state later.
	notifyQueueFull  uint32
	notifySent       uint32 // reservations
	notifyAccepted   uint32 // successful submissions
	notifyCompleted  uint32
	notifyCreditWait uint32
	notifyLastReport time.Time
	// Restart attempt is deferred outside the SoftDevice event handler.
	restartAdvertisingAfter time.Time
	channels                [8]uint16

	remote ParaRemote
}

type ParaRemote struct {
	// remote commands
	orientationReset bool
	factoryReset     bool
	reboot           bool

	// remote configuration
	nameChanged        bool
	nameValue          [16]byte
	nameLength         byte
	axisMappingChanged bool
	axisMappingValue   [3]byte
}

type CallbackHandler interface {
	// bluetooth connection events
	OnConnect()
	OnDisconnect()

	// remote commands
	OnOrientationReset()
	OnReboot()
	OnFactoryReset()

	// remote configuration
	OnDeviceNameChange(name string)
	OnAxisMappingChange(mapping [3]byte)
}

func NewPara(name string, axisMapping [3]byte, callbackHandler CallbackHandler) *Para {
	para := Para{
		adapter:         bluetooth.DefaultAdapter,
		callbackHandler: callbackHandler,
		paired:          false,
		channels:        [8]uint16{1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500},
		remote: ParaRemote{
			nameChanged:        false,
			nameLength:         byte(len(name)),
			axisMappingChanged: false,
			axisMappingValue:   axisMapping,
		},
	}
	for i := 0; i < len(name) && i < 16; i++ {
		para.remote.nameValue[i] = byte(name[i])
	}
	return &para
}

func (t *Para) Start() string {
	t.adapter.Enable()

	setDeviceName(t.remote.nameValue[:t.remote.nameLength])

	sysid := bluetooth.CharacteristicConfig{
		Handle: nil,
		UUID:   bluetooth.CharacteristicUUIDSystemID,
		Value:  []byte{0xF1, 0x63, 0x1B, 0xB0, 0x6F, 0x80, 0x28, 0xFE},
		Flags:  bluetooth.CharacteristicReadPermission,
	}

	manufacturer := bluetooth.CharacteristicConfig{
		Handle: nil,
		UUID:   bluetooth.CharacteristicUUIDManufacturerNameString,
		Value:  []byte{'S', 'k', 'y', 'G', 'a', 'd', 'g', 'e', 't', 's', ' ', 'A', 'B'},
		Flags:  bluetooth.CharacteristicReadPermission,
	}

	ieee := bluetooth.CharacteristicConfig{
		Handle: nil,
		UUID:   bluetooth.CharacteristicUUIDIEEE1107320601RegulatoryCertificationDataList,
		Value:  []byte{0xFE, 0x00, 0x65, 0x78, 0x70, 0x65, 0x72, 0x69, 0x6D, 0x65, 0x6E, 0x74, 0x61, 0x6C},
		Flags:  bluetooth.CharacteristicReadPermission,
	}

	pnpid := bluetooth.CharacteristicConfig{
		Handle: nil,
		UUID:   bluetooth.CharacteristicUUIDPnPID,
		Value:  []byte{0x01, 0x0D, 0x00, 0x00, 0x00, 0x10, 0x01},
		Flags:  bluetooth.CharacteristicReadPermission,
	}

	t.adapter.AddService(&bluetooth.Service{
		UUID: bluetooth.ServiceUUIDDeviceInformation,
		Characteristics: []bluetooth.CharacteristicConfig{
			sysid, manufacturer, ieee, pnpid,
		},
	})

	fff1 := bluetooth.CharacteristicConfig{
		Handle: nil,
		UUID:   bluetooth.New16BitUUID(0xFFF1),
		Value:  []byte{0x01},
		Flags:  bluetooth.CharacteristicReadPermission | bluetooth.CharacteristicWritePermission,
	}

	fff2 := bluetooth.CharacteristicConfig{
		Handle: nil,
		UUID:   bluetooth.New16BitUUID(0xFFF2),
		Value:  []byte{0x02},
		Flags:  bluetooth.CharacteristicReadPermission,
	}

	fff3 := bluetooth.CharacteristicConfig{
		Handle: nil,
		UUID:   bluetooth.New16BitUUID(0xFFF3),
		Value:  []byte{},
		Flags:  bluetooth.CharacteristicWriteWithoutResponsePermission,
	}

	fff5 := bluetooth.CharacteristicConfig{
		Handle: nil,
		UUID:   bluetooth.New16BitUUID(0xFFF5),
		Value:  []byte{},
		Flags:  bluetooth.CharacteristicReadPermission,
	}

	fff6 := bluetooth.CharacteristicConfig{
		Handle: &t.fff6Handle,
		UUID:   bluetooth.New16BitUUID(0xFFF6),
		Value:  []byte{},
		Flags:  bluetooth.CharacteristicWriteWithoutResponsePermission | bluetooth.CharacteristicNotifyPermission,
	}

	// Commands
	// R - reset orientation, compatible with Cliff's HT reset button
	// F - factory reset
	// B - reboot device
	commandHandler := func(client bluetooth.Connection, offset int, value []byte) {
		if len(value) == 1 && value[0] == CMD_ORIENTATION_RESET {
			t.remote.orientationReset = true
		}
		if len(value) == 1 && value[0] == CMD_FACTORY_RESET {
			t.remote.factoryReset = true
		}
		if len(value) == 1 && value[0] == CMD_REBOOT {
			t.remote.reboot = true
		}
	}
	charCmd := bluetooth.CharacteristicConfig{
		Handle:     nil,
		UUID:       bluetooth.New16BitUUID(CHAR_COMMANDS),
		Value:      []byte{},
		Flags:      bluetooth.CharacteristicWritePermission,
		WriteEvent: commandHandler,
	}
	// Compatibility with Cliff's Head Tracker
	charCmdCompat := bluetooth.CharacteristicConfig{
		Handle:     nil,
		UUID:       bluetooth.New16BitUUID(CHAR_COMMANDS_COMPAT),
		Value:      []byte{},
		Flags:      bluetooth.CharacteristicWritePermission,
		WriteEvent: commandHandler,
	}

	charDeviceName := bluetooth.CharacteristicConfig{
		Handle: nil,
		UUID:   bluetooth.New16BitUUID(CHAR_DATA_DEVICE_NAME),
		Value:  t.remote.nameValue[:t.remote.nameLength],
		Flags:  bluetooth.CharacteristicReadPermission | bluetooth.CharacteristicWritePermission,
		WriteEvent: func(client bluetooth.Connection, offset int, value []byte) {
			if len(value) == 0 {
				return
			}
			n := 0
			for n < len(t.remote.nameValue) && n < len(value) {
				t.remote.nameValue[n] = value[n]
				n++
			}
			t.remote.nameLength = byte(n)
			t.remote.nameChanged = true
		},
	}

	charAxisMapping := bluetooth.CharacteristicConfig{
		Handle: nil,
		UUID:   bluetooth.New16BitUUID(CHAR_DATA_AXIS_MAPPING),
		Value:  t.remote.axisMappingValue[:],
		Flags:  bluetooth.CharacteristicReadPermission | bluetooth.CharacteristicWritePermission,
		WriteEvent: func(client bluetooth.Connection, offset int, value []byte) {
			if len(value) != 3 {
				return
			}
			for i := 0; i < 3; i++ {
				t.remote.axisMappingValue[i] = value[i] & 0b00110111 // mask out unused bits
			}
			t.remote.axisMappingChanged = true
		},
	}

	t.adapter.AddService(&bluetooth.Service{
		UUID: bluetooth.New16BitUUID(0xFFF0),
		Characteristics: []bluetooth.CharacteristicConfig{
			fff1,
			fff2,
			fff3,
			fff5,
			fff6,            // channels data
			charCmd,         // runtime commands from client
			charCmdCompat,   // for compatibility with Cliff's HT
			charDeviceName,  // device name
			charAxisMapping, // axis mapping
		},
	})

	t.adv = t.adapter.DefaultAdvertisement()
	t.adv.Configure(bluetooth.AdvertisementOptions{
		LocalName:    string(t.remote.nameValue[:t.remote.nameLength]),
		ServiceUUIDs: []bluetooth.UUID{bluetooth.New16BitUUID(0xFFF0)},
	})
	t.adv.Start()

	// Completion reports how many BLE notifications have left SoftDevice.
	// Keep callback minimal; Bluetooth APIs must not be used in event context.
	t.adapter.SetNotificationCompleteHandler(func(count uint16) {
		atomic.AddUint32(&t.notifyCompleted, uint32(count))
	})

	t.adapter.SetConnectHandler(func(device bluetooth.Device, connected bool) {
		if connected {
			atomic.StoreUint32(&t.notifySent, 0)
			atomic.StoreUint32(&t.notifyAccepted, 0)
			atomic.StoreUint32(&t.notifyCompleted, 0)
			t.notifyCreditWait = 0
			t.sendAfter = time.Now().Add(1 * time.Second) // wait for 1 second before sending data
			setSoftDeviceSystemAttributes()               // force enable notify for fff6
			atomic.AddUint32(&t.notifySent, 1)
			if _, err := t.fff6Handle.Write(bootBuffer); err != nil {
				atomic.AddUint32(&t.notifyCompleted, 1) // release reserved credit
			} else {
				atomic.AddUint32(&t.notifyAccepted, 1)
			} // send '\r\n', it helps remote master switch to receiveTrainer state
			t.paired = true
			t.callbackHandler.OnConnect()
		} else {
			t.paired = false
			// TinyGo v0.13.0 attempts an automatic SoftDevice advertising
			// restart, but its return code is not checked. Schedule a
			// fallback attempt in the ordinary task loop instead of
			// calling BLE APIs from this interrupt/event callback.
			t.restartAdvertisingAfter = time.Now().Add(1500 * time.Millisecond)
			t.callbackHandler.OnDisconnect()
		}
	})

	go func() {
		ticker := time.NewTicker(20 * time.Millisecond)
		for range ticker.C {

			t.update()

			if !t.paired && !t.restartAdvertisingAfter.IsZero() && time.Now().After(t.restartAdvertisingAfter) {
				t.restartAdvertisingAfter = time.Time{}
				// If the SoftDevice already restarted advertising, this may
				// report an already-active/invalid-state result. Log it so
				// we can distinguish that from genuine restart failures.
				if err := t.adv.Start(); err != nil {
					println("Bluetooth advertising fallback:", err.Error())
				} else {
					println("Bluetooth advertising restarted after disconnect")
				}
			}

			if t.remote.orientationReset {
				t.remote.orientationReset = false
				t.callbackHandler.OnOrientationReset()
			}
			if t.remote.factoryReset {
				t.remote.factoryReset = false
				t.callbackHandler.OnFactoryReset()
			}
			if t.remote.reboot {
				t.remote.reboot = false
				t.callbackHandler.OnReboot()
			}
			if t.remote.nameChanged {
				t.remote.nameChanged = false
				nameBytes := t.remote.nameValue[:t.remote.nameLength]
				setDeviceName(nameBytes)
				t.callbackHandler.OnDeviceNameChange(string(nameBytes))
				// update advertisement name
				// can't do it in the write handler or on disconnect due to allocation restrictions in interrupt handlers
				t.adv.Configure(bluetooth.AdvertisementOptions{
					LocalName:    string(nameBytes),
					ServiceUUIDs: []bluetooth.UUID{bluetooth.New16BitUUID(0xFFF0)},
				})
			}
			if t.remote.axisMappingChanged {
				t.remote.axisMappingChanged = false
				t.callbackHandler.OnAxisMappingChange(t.remote.axisMappingValue)
			}
		}
	}()

	addr, _ := t.adapter.Address()
	return addr.MAC.String()

}

func (t *Para) update() {
	if !t.paired {
		return
	}
	if !time.Now().After(t.sendAfter) {
		return
	}
	if time.Since(t.notifyLastReport) >= 10*time.Second {
		t.notifyLastReport = time.Now()
		println("FFF6 credit: full", t.notifyQueueFull, "wait", t.notifyCreditWait,
			"accepted", atomic.LoadUint32(&t.notifyAccepted), "complete", atomic.LoadUint32(&t.notifyCompleted))
	}
	// A single outstanding notification bounds SoftDevice queueing latency.
	// Any position generated while busy is obsolete by the next send tick.
	if atomic.LoadUint32(&t.notifySent) != atomic.LoadUint32(&t.notifyCompleted) {
		t.notifyCreditWait++
		return
	}
	size := t.encode()
	atomic.AddUint32(&t.notifySent, 1) // reserve credit before calling into SoftDevice
	n, err := t.fff6Handle.Write(t.buffer[:size])
	if err == nil {
		atomic.AddUint32(&t.notifyAccepted, 1)
		return
	}
	atomic.AddUint32(&t.notifyCompleted, 1) // undo reservation on failed send
	// Nordic NRF_ERROR_RESOURCES (19) is a normal backpressure signal:
	// too many notifications are pending. Never retry this sample.
	if sdErr, ok := err.(bluetooth.Error); ok && sdErr == bluetooth.Error(19) {
		t.notifyQueueFull++
		return
	}
	println("FFF6 write error:", err.Error(), n)
}

func (p *Para) SetChannel(n int, v uint16) {
	p.channels[n] = v
}

// -- PARA Protocol ------------------------------------------------------------

// 2 + 8(max 16) + 2

const START_STOP byte = 0x7E
const BYTE_STUFF byte = 0x7D
const STUFF_MASK byte = 0x20

// Escapes bytes that equal to START_STOP and updates CRC
func (t *Para) push(b byte, bufferIndex *byte, crc *byte) {
	*crc ^= b
	if b == START_STOP || b == BYTE_STUFF {
		t.buffer[*bufferIndex] = BYTE_STUFF
		*bufferIndex++
		b ^= STUFF_MASK
	}
	t.buffer[*bufferIndex] = b
	*bufferIndex++
}

// Encodes the channels array to a para trainer packet (adapted from OpenTX source code)
func (t *Para) encode() byte {

	var bufferIndex byte = 0
	var crc byte = 0x00

	t.buffer[bufferIndex] = START_STOP
	bufferIndex++
	t.push(0x80, &bufferIndex, &crc)
	for channel := 0; channel < 8; channel += 2 {
		channelValue1 := t.channels[channel]
		channelValue2 := t.channels[channel+1]
		t.push(byte(channelValue1&0x00ff), &bufferIndex, &crc)
		t.push(byte((channelValue1&0x0f00)>>4)+byte((channelValue2&0x00f0)>>4), &bufferIndex, &crc)
		t.push(byte((channelValue2&0x000f)<<4)+byte((channelValue2&0x0f00)>>8), &bufferIndex, &crc)
	}
	t.buffer[bufferIndex] = crc
	t.buffer[bufferIndex+1] = START_STOP

	return bufferIndex + 2
}
