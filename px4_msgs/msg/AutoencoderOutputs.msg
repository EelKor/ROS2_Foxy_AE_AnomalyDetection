uint64 timestamp                    # time since system start (microseconds)

# Autoencoder Outputs
float32 reconstruction_mse        # reconstruction_error(MSE) of autoencoder
float32[12] x                     # Input Value
float32[12] x_hat                 # Reconstructed Value
