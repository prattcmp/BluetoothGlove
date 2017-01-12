var noble = require('noble');
var sleep = require('sleep');

// We need this for sleep to work
require('events').EventEmitter.prototype._maxListeners = 1000;

var peripheralName = 'HapticGloveB';
var motorServiceUuid = '15db5d2050d44370a439754e7182cb54';
var motorCharacteristicUuid = '15db5d2150d44370a439754e7182cb54';

noble.on('stateChange', function(state) {
	if (state === 'poweredOn') {
		//
		// Once the BLE radio has been powered on, it is possible
		// to begin scanning for services. Pass an empty array to
		// scan for all services (uses more time and power).
		//
		console.log('scanning...');
		noble.startScanning();
	}
	else {
		console.log('bluetooth disabled');
		noble.stopScanning();
	}
})

var motorService = null;
var motorCharacteristic = null;
 
noble.on('discover', function(peripheral) {
	if (peripheral.advertisement.localName === peripheralName) {
	// we found our peripheral, stop scanning
	noble.stopScanning();
	
	connect(peripheral);
	}
});

function connect(peripheral) {
	peripheral.connect(function(err) {
		peripheral.discoverServices([motorServiceUuid], function(err, services) {
			services.forEach(function(service) {
				// This must be the service we were looking for.
				// So, discover its characteristics.
				service.discoverCharacteristics([], function(err, characteristics) {
					characteristics.forEach(function(characteristic) {
						//
						// Loop through each characteristic and match them to the
						// UUIDs that we know about.
						//
						if (motorCharacteristicUuid == characteristic.uuid) {
								motorCharacteristic = characteristic;
							}
					})
					//
					// Check to see if we found our characteristic.
					//
					if (motorCharacteristic) {
						//
						// We did, so run our tests
						//
						runTests();
					}
					else {
						console.log('missing characteristics');
					}
				})
			})
		})
	})
}

function runMotor(motor, intensity, duration) {
	var packet = Buffer.alloc(3);
	packet.writeUInt8(motor, 0);
	packet.writeUInt8(intensity, 1);
	packet.writeUInt8(duration, 2);

	motorCharacteristic.write(packet, false, function(error) { console.log(error ? error : "") });
}

function runTests() {
	// Run all motors
	console.log("Running all motors...");
	sleep.sleep(1);
	for (i = 0; i < 5; i++) {
		runMotor(7, 100, 100);
		sleep.sleep(1);
	}
	
	// Run each motor
	console.log("Running each motor...");
	for (i = 1; i < 7; i++) {
		runMotor(i, 100, 100);
		console.log("Motor " + i);
		sleep.sleep(2);
	}

	// Run motors at different intensities
	console.log("Running motors at set intensities...");
	for (i = 0; i < 100; i += 5) {
		runMotor(7, i, 10);
		sleep.usleep(9000);
	}
}
