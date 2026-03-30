#!/usr/bin/env python3
"""
PSD7168 Power Supply Controller - STANDALONE (No External Dependencies)
========================================================================

Control PSD7168 via Socket (TCP/IP port 5025) and optionally read CCID via serial.
Uses only Python standard library (socket, serial, csv, logging).

Usage:
    # Simple voltage test (no CCID needed)
   python3 psd7168_control_standalone.py --psd-ip 192.168.1.170
    
    # With CCID Modbus recording
    python3 psd7168_control_standalone.py --psd-ip 192.168.1.170 \\
                                          --ccid-port /dev/ttyUSB0 \\
                                          --start-v 0 --stop-v 10 \\
                                          --step-v 1 --output results.csv

"""

import socket
import time
import csv
import logging
import argparse
import sys
import struct
from datetime import datetime
from typing import Optional, Tuple, List, Dict

# Try to import serial (usually available in Python stdlib as pyserial)
try:
    import serial
    HAS_SERIAL = True
except ImportError:
    HAS_SERIAL = False


# ============================================================================
# PSD7168 Controller (Socket/TCP-IP)
# ============================================================================

class PSD7168Controller:
    """
    Control PSD7168 via Socket (TCP/IP port 5025).
    Supports SCPI commands: VOLT, CURR, MEAS, OUTPut, etc.
    """
    
    def __init__(self, ip_address: str, port: int = 5025, timeout: float = 2.0):
        self.ip_address = ip_address
        self.port = port
        self.timeout = timeout
        self.socket = None
        self.logger = logging.getLogger("PSD7168")
        
    def connect(self) -> bool:
        """Establish TCP/IP connection to PSD7168."""
        try:
            self.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.socket.settimeout(self.timeout)
            self.socket.connect((self.ip_address, self.port))
            self.logger.info(f"✓ Connected to PSD7168 at {self.ip_address}:{self.port}")
            
            # Verify with IDN query
            idn = self.query("*IDN?")
            self.logger.info(f"  Device: {idn.strip()}")
            return True
        except Exception as e:
            self.logger.error(f"✗ Connection failed: {e}")
            return False
    
    def disconnect(self) -> None:
        """Close connection."""
        if self.socket:
            try:
                self.socket.close()
                self.logger.info("✓ Disconnected from PSD7168")
            except:
                pass
    
    def write(self, command: str) -> bool:
        """Send command (no response)."""
        try:
            self.socket.sendall((command + "\n").encode('utf-8'))
            self.logger.debug(f"TX: {command}")
            time.sleep(0.05)
            return True
        except Exception as e:
            self.logger.error(f"Write failed: {e}")
            return False
    
    def query(self, command: str) -> str:
        """Send command and read response."""
        try:
            self.socket.sendall((command + "\n").encode('utf-8'))
            time.sleep(0.1)
            response = self.socket.recv(4096).decode('utf-8')
            self.logger.debug(f"TX: {command} -> RX: {response.strip()}")
            return response
        except Exception as e:
            self.logger.error(f"Query failed: {e}")
            return ""
    
    def set_voltage(self, voltage: float) -> bool:
        """Set output voltage (0-16V)."""
        if not (0 <= voltage <= 16):
            self.logger.error(f"Voltage {voltage}V out of range (0-16V)")
            return False
        return self.write(f"VOLT {voltage:.3f}")
    
    def set_current_limit(self, current: float) -> bool:
        """Set current limit (0-8A)."""
        if not (0 <= current <= 8):
            self.logger.error(f"Current {current}A out of range (0-8A)")
            return False
        return self.write(f"CURRent {current:.3f}")
    
    def set_output(self, enabled: bool) -> bool:
        """Enable/disable output."""
        state = "ON" if enabled else "OFF"
        return self.write(f"OUTPut CH1, {state}")
    
    def get_voltage(self) -> Optional[float]:
        """Query actual output voltage."""
        try:
            response = self.query("MEASure:VOLTage?")
            return float(response.strip())
        except ValueError:
            return None
    
    def get_current(self) -> Optional[float]:
        """Query actual output current."""
        try:
            response = self.query("MEASure:CURRent?")
            return float(response.strip())
        except ValueError:
            return None
    
    def get_power(self) -> Optional[float]:
        """Query actual output power."""
        try:
            response = self.query("MEASure:POWEr?")
            return float(response.strip())
        except ValueError:
            return None


# ============================================================================
# CCID Modbus RTU Reader (Manual Implementation - No pymodbus Required)
# ============================================================================

class CCIDModbusReader:
    """
    Read CCID via serial Modbus RTU (manual implementation).
    Supports 9600 baud 8N1. Manually encodes/decodes Modbus frames.
    """
    
    def __init__(self, port: str = "/dev/ttyUSB0", baudrate: int = 9600, 
                 slave_id: int = 1, timeout: float = 1.0):
        self.port = port
        self.baudrate = baudrate
        self.slave_id = slave_id
        self.timeout = timeout
        self.ser = None
        self.logger = logging.getLogger("CCID_Modbus")
        
        if not HAS_SERIAL:
            self.logger.warning("serial module not available (pyserial). CCID disabled.")
    
    def connect(self) -> bool:
        """Establish serial connection."""
        if not HAS_SERIAL:
            self.logger.error("Cannot connect: serial module not available")
            return False
        
        try:
            self.ser = serial.Serial(
                port=self.port,
                baudrate=self.baudrate,
                bytesize=8,
                parity=serial.PARITY_NONE,
                stopbits=1,
                timeout=self.timeout
            )
            self.logger.info(f"✓ Connected to CCID at {self.port} (9600 baud, 8N1)")
            return True
        except Exception as e:
            self.logger.error(f"✗ Serial connection failed: {e}")
            return False
    
    def disconnect(self) -> None:
        """Close serial connection."""
        if self.ser:
            try:
                self.ser.close()
                self.logger.info("✓ Disconnected from CCID")
            except:
                pass
    
    @staticmethod
    def _crc16(data: bytes) -> int:
        """Calculate CRC-16 (Modbus RTU)."""
        crc = 0xFFFF
        for byte in data:
            crc ^= byte
            for _ in range(8):
                if crc & 1:
                    crc = (crc >> 1) ^ 0xA001
                else:
                    crc >>= 1
        return crc
    
    def _build_request(self, register: int, count: int = 2) -> bytes:
        """Build Modbus RTU read holding registers request."""
        # Function 03: Read Holding Registers
        frame = bytes([
            self.slave_id,           # Slave ID
            0x03,                    # Function: Read Holding Registers
            (register >> 8) & 0xFF,  # Register address (high byte)
            register & 0xFF,         # Register address (low byte)
            0x00,                    # Count (high byte)
            count & 0xFF             # Count (low byte)
        ])
        
        crc = self._crc16(frame)
        frame += bytes([crc & 0xFF, (crc >> 8) & 0xFF])  # CRC (low, high)
        return frame
    
    def _parse_response(self, data: bytes) -> Optional[List[int]]:
        """Parse Modbus RTU response."""
        if len(data) < 5:
            self.logger.error(f"Response too short: {len(data)} bytes")
            return None
        
        # Verify CRC
        received_crc = (data[-1] << 8) | data[-2]
        calculated_crc = self._crc16(data[:-2])
        if received_crc != calculated_crc:
            self.logger.error(f"CRC mismatch: got 0x{received_crc:04X}, expected 0x{calculated_crc:04X}")
            return None
        
        # Parse registers
        byte_count = data[2]
        registers = []
        for i in range(byte_count // 2):
            reg = (data[3 + i*2] << 8) | data[3 + i*2 + 1]
            registers.append(reg)
        
        return registers
    
    def read_registers(self, address: int, count: int = 2) -> Optional[List[int]]:
        """Read holding registers from CCID."""
        if not self.ser:
            self.logger.error("Not connected")
            return None
        
        try:
            # Send request
            request = self._build_request(address, count)
            self.ser.write(request)
            self.logger.debug(f"TX: {request.hex()}")
            
            # Read response
            response = self.ser.read(5 + count * 2)
            self.logger.debug(f"RX: {response.hex()}")
            
            if not response:
                self.logger.error("No response from CCID")
                return None
            
            return self._parse_response(response)
        except Exception as e:
            self.logger.error(f"Read failed: {e}")
            return None
    
    def read_leakage_current(self) -> Optional[float]:
        """
        Read leakage current from register 0x0000 (Q16.16 fixed-point).
        Assumes 2 registers (32-bit value) with format: [High16, Low16].
        """
        registers = self.read_registers(0x0000, count=2)
        if not registers or len(registers) < 2:
            return None
        
        try:
            # Combine registers: 32-bit value
            raw_value = (registers[0] << 16) | registers[1]
            
            # Convert from Q16.16 fixed-point to float (mA)
            current_ma = raw_value / 65536.0
            self.logger.debug(f"Raw: 0x{raw_value:08X} -> {current_ma:.4f} mA")
            return current_ma
        except Exception as e:
            self.logger.error(f"Conversion failed: {e}")
            return None
    
    def read_status(self) -> Optional[int]:
        """Read status register (0x0001)."""
        registers = self.read_registers(0x0001, count=1)
        return registers[0] if registers else None
    
    def read_diagnostic(self, register: int) -> Optional[int]:
        """Read diagnostic register (0x0010-0x001A)."""
        registers = self.read_registers(register, count=1)
        return registers[0] if registers else None


# ============================================================================
# Test Harness
# ============================================================================

class TestHarness:
    """Coordinate PSD7168 + CCID for synchronized voltage sweep + logging."""
    
    def __init__(self, psd_ip: str, ccid_port: Optional[str] = None):
        self.psd = PSD7168Controller(psd_ip)
        self.ccid = CCIDModbusReader(ccid_port) if ccid_port else None
        self.logger = logging.getLogger("TestHarness")
        self.results = []
    
    def setup(self) -> bool:
        """Initialize devices."""
        self.logger.info("Setting up devices...")
        
        if not self.psd.connect():
            return False
        
        # Safe defaults
        self.psd.set_current_limit(0.5)
        self.psd.set_voltage(0.0)
        
        if self.ccid:
            if self.ccid.connect():
                self.logger.info("CCID connected")
            else:
                self.logger.warning("CCID not available (continuing without Modbus)")
                self.ccid = None
        
        return True
    
    def teardown(self) -> None:
        """Safe shutdown."""
        self.logger.info("Shutting down...")
        try:
            self.psd.set_output(False)
            time.sleep(0.2)
        except:
            pass
        self.psd.disconnect()
        if self.ccid:
            self.ccid.disconnect()
    
    def voltage_sweep(self, start_v: float, stop_v: float, step_v: float,
                     dwell_time: float = 1.0, samples_per_step: int = 5) -> List[Dict]:
        """Perform voltage sweep with CCID recording."""
        self.logger.info(f"Starting sweep: {start_v}V → {stop_v}V, {step_v}V steps")
        
        results = []
        voltage = start_v
        
        try:
            self.psd.set_output(True)
            time.sleep(0.5)
            
            while (step_v > 0 and voltage <= stop_v) or (step_v < 0 and voltage >= stop_v):
                if not self.psd.set_voltage(voltage):
                    break
                
                self.logger.info(f"  Voltage: {voltage:.1f}V, dwell: {dwell_time}s")
                time.sleep(dwell_time)
                
                v_actual = self.psd.get_voltage()
                i_supply = self.psd.get_current()
                
                ccid_readings = []
                if self.ccid:
                    for _ in range(samples_per_step):
                        ccid_i = self.ccid.read_leakage_current()
                        if ccid_i is not None:
                            ccid_readings.append(ccid_i)
                        time.sleep(0.1)
                
                ccid_avg = sum(ccid_readings) / len(ccid_readings) if ccid_readings else None
                
                record = {
                    'timestamp': datetime.now().isoformat(),
                    'voltage_set': voltage,
                    'voltage_actual': v_actual,
                    'input_current_a': i_supply,
                    'ccid_leakage_ma': ccid_avg,
                    'ccid_samples': len(ccid_readings)
                }
                results.append(record)
                
                self.logger.info(f"    V={v_actual:.3f}V, I={i_supply:.4f}A, CCID={ccid_avg:.4f}mA")
                
                voltage += step_v
        
        finally:
            self.psd.set_output(False)
        
        self.results = results
        return results
    
    def save_csv(self, filename: str) -> None:
        """Save results to CSV."""
        if not self.results:
            self.logger.warning("No results to save")
            return
        
        try:
            with open(filename, 'w', newline='') as f:
                writer = csv.DictWriter(f, fieldnames=self.results[0].keys())
                writer.writeheader()
                writer.writerows(self.results)
            
            self.logger.info(f"✓ Saved {len(self.results)} results to {filename}")
        except Exception as e:
            self.logger.error(f"Save failed: {e}")


# ============================================================================
# CLI
# ============================================================================

def main():
    parser = argparse.ArgumentParser(
        description="PSD7168 Voltage Control + CCID Leakage Current Recording",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  # Simple voltage test (no CCID)
  %(prog)s --psd-ip 192.168.1.100

  # Full sweep with CCID recording
  %(prog)s --psd-ip 192.168.1.100 --ccid-port /dev/ttyUSB0 \\
           --start-v 0 --stop-v 12 --step-v 1 --dwell-time 2
        """
    )
    
    parser.add_argument("--psd-ip", required=True, 
                       help="IP address of PSD7168 (required)")
    parser.add_argument("--ccid-port", default=None,
                       help="Serial port for CCID (e.g., /dev/ttyUSB0 or COM3)")
    parser.add_argument("--start-v", type=float, default=0.0,
                       help="Start voltage (default: 0V)")
    parser.add_argument("--stop-v", type=float, default=10.0,
                       help="Stop voltage (default: 10V)")
    parser.add_argument("--step-v", type=float, default=1.0,
                       help="Voltage step (default: 1V)")
    parser.add_argument("--dwell-time", type=float, default=1.0,
                       help="Dwell time per step (default: 1s)")
    parser.add_argument("--samples", type=int, default=5,
                       help="CCID samples per step (default: 5)")
    parser.add_argument("--output", default="psd7168_test_results.csv",
                       help="Output CSV file (default: psd7168_test_results.csv)")
    parser.add_argument("--log-level", default="INFO",
                       choices=["DEBUG", "INFO", "WARNING", "ERROR"],
                       help="Logging level (default: INFO)")
    
    args = parser.parse_args()
    
    # Setup logging
    logging.basicConfig(
        level=getattr(logging, args.log_level),
        format='%(asctime)s [%(levelname)s] %(name)s: %(message)s'
    )
    logger = logging.getLogger("main")
    
    # Run test
    harness = TestHarness(args.psd_ip, args.ccid_port)
    
    if not harness.setup():
        logger.error("Setup failed")
        return 1
    
    try:
        harness.voltage_sweep(
            start_v=args.start_v,
            stop_v=args.stop_v,
            step_v=args.step_v,
            dwell_time=args.dwell_time,
            samples_per_step=args.samples
        )
        harness.save_csv(args.output)
        logger.info("✓ Test completed successfully")
    except KeyboardInterrupt:
        logger.warning("Test interrupted")
    except Exception as e:
        logger.error(f"Test failed: {e}", exc_info=True)
        return 1
    finally:
        harness.teardown()
    
    return 0


if __name__ == "__main__":
    sys.exit(main())
