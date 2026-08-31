# CRSFTelemetryAdapter
Hobbywing Telemetry Adapter for CRSF

Uses ELRS CRSF protocol to show telemetry data of a hobbywing XeRun or EzRun ESC (RPM, external voltage, ESC temperature, Motor temperature) on your OpenTX or EdgeTX transmitter. Just connect Serial1 to one of the CRSF RX/TX Ports of your ELRS receiver and Serial2 to the programmer port of your ESC.

The adapter is powered by the receiver. Be sure the BEC voltage of your ESC is not higher then 6V. Use of a diode is recommeded.

Thanks to @AlfredoSystems for the CRSF library
