#include "simple_cipher.h"

int SimpleDecrypt( const unsigned char key128[16], const unsigned char ciphertext[16], unsigned char plaintext[16] )
{
    int i;

    for (i = 0; i < 16; i++) {
        plaintext[i] = ciphertext[i] ^ key128[i];
    }

    return 0;
}

