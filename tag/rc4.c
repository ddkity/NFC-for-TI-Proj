
#include "rc4.h"

int Rc4Decrypt( const unsigned char* key, unsigned int keyLen, unsigned int dataLength, const unsigned char* ciphertext, unsigned char* plaintext )
{
    int ret = Rc4Encrypt(key, keyLen, dataLength, ciphertext, plaintext);
    return ret;
}

int Rc4Encrypt( const unsigned char* key, unsigned int keyLen, unsigned int dataLength, const unsigned char* plaintext, unsigned char* ciphertext )
{
    Rc4Context ctx;
    Rc4Setup( &ctx, key, keyLen );
    Rc4Encryption(&ctx, dataLength, plaintext, ciphertext);
    return 0;
}

/*
 * ARC4 key schedule
 */
void Rc4Setup( Rc4Context* ctx, const unsigned char* key, unsigned int keyLen )
{
    int i, j, a;
    unsigned int k;
    unsigned char* m;

    ctx->x = 0;
    ctx->y = 0;
    m = ctx->m;

    for( i = 0; i < 256; i++ )
        m[i] = (unsigned char) i;

    j = k = 0;

    for( i = 0; i < 256; i++, k++ )
    {
        if( k >= keyLen ) k = 0;

        a = m[i];
        j = ( j + a + key[k] ) & 0xFF;
        m[i] = m[j];
        m[j] = (unsigned char) a;
    }
}

/*
 * ARC4 cipher function
 */
int Rc4Encryption( Rc4Context* ctx, unsigned int length, const unsigned char* input,
                unsigned char* output )
{
    int x, y, a, b;
    unsigned int i;
    unsigned char* m;

    x = ctx->x;
    y = ctx->y;
    m = ctx->m;

    for( i = 0; i < length; i++ )
    {
        x = ( x + 1 ) & 0xFF; a = m[x];
        y = ( y + a ) & 0xFF; b = m[y];

        m[x] = (unsigned char) b;
        m[y] = (unsigned char) a;

        output[i] = (unsigned char)
            ( input[i] ^ m[(unsigned char)( a + b )] );
    }

    ctx->x = x;
    ctx->y = y;

    return( 0 );
}

