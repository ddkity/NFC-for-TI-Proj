
#ifndef RC4_H_
#define RC4_H_

#ifdef  __cplusplus
extern "C" {
#endif

/**
 * \brief          ARC4 context structure
 */
typedef struct
{
    int x;                      /*!< permutation index */
    int y;                      /*!< permutation index */
    unsigned char m[256];       /*!< permutation table */
}Rc4Context;

int Rc4Encrypt( const unsigned char* key, unsigned int keyLen, unsigned int dataLength, const unsigned char* plaintext, unsigned char* ciphertext );
int Rc4Decrypt( const unsigned char* key, unsigned int keyLen, unsigned int dataLength, const unsigned char* ciphertext, unsigned char* plaintext );

/**
 * \brief          ARC4 key schedule
 *
 * \param ctx      ARC4 context to be initialized
 * \param key      the secret key
 * \param keyLen   length of the key
 */
void Rc4Setup( Rc4Context* ctx, const unsigned char* key, unsigned int keyLen );

/**
 * \brief          ARC4 cipher function
 *
 * \param ctx      ARC4 context
 * \param length   length of the input data
 * \param input    buffer holding the input data
 * \param output   buffer for the output data
 *
 * \return         0 if successful
 */
int Rc4Encryption( Rc4Context* ctx, unsigned int length, const unsigned char* input,
                unsigned char* output );
#ifdef  __cplusplus
}
#endif

#endif /* RC4_H_ */
