import numpy as np
from scipy.signal import firwin

def generate_c_array(name, data):
    """Formats a NumPy array as a C-style array string."""
    array_str = ", ".join([f"{x:.6f}f" for x in data])
    return f"const float {name}[{len(data)}] = {{ {array_str} }};"

def main():
    # FIR Filter Parameters
    sample_rate = 500.0   # Hz (IMU sampling rate: 1kHz gyro / (1+SMPLRT_DIV=1))
    cutoff_hz = 5.0       # Hz (Low-pass cutoff frequency for hand gestures)
    num_taps = 31         # Must match FIR_TAPS in signal_processing.h
    
    # Generate Low-pass FIR coefficients using Hamming window
    coeffs = firwin(num_taps, cutoff_hz, fs=sample_rate)
    
    # Print the C array definition
    print("// Copy this into Core/Src/signal_processing.c")
    print(generate_c_array("FIR_COEFFS_LOWPASS", coeffs))

if __name__ == "__main__":
    main()
