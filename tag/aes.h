#ifndef __AES_H__
#define __AES_H__
#ifdef  __cplusplus
extern "C" {
#endif
enum AES_OPERATOR_TYPE
{
    AES_ENCRYPT = 0,
    AES_DECRYPT = 1
};

enum AES_ERROR
{
    ERROR_AES_INVALID_KEY_LENGTH = -0x0020,  /**< Invalid key length. */
    ERROR_AES_INVALID_INPUT_LENGTH = -0x0022  /**< Invalid data input length. */
};

int Aes128EncryptECB( const unsigned char key128[16], const unsigned char plaintext[16], unsigned char ciphertext[16] );
int Aes128DecryptECB( const unsigned char key128[16], const unsigned char ciphertext[16], unsigned char plaintext[16] );
int Aes128EncryptCBC( const unsigned char key128[16], const unsigned char iv[16], unsigned int length, const unsigned char* plaintext, unsigned char* ciphertext );
int Aes128DecryptCBC( const unsigned char key128[16], const unsigned char iv[16], unsigned int length,  const unsigned char* ciphertext, unsigned char* plaintext );

/**
 * \brief          AES context structure
 */
typedef struct
{
    int nr;                     /*!<  number of rounds  */
    unsigned long *rk;          /*!<  AES round keys    */
    unsigned long buf[68];      /*!<  unaligned data    */
}AesContext;

/**
 * \brief          AES key schedule (encryption)
 *
 * \param ctx      AES context to be initialized
 * \param key      encryption key
 * \param keysize  must be 128, 192 or 256
 *
 * \return         0 if successful, or ERROR_AES_INVALID_KEY_LENGTH
 */
int AesSetkeyEnc( AesContext *ctx, const unsigned char *key, unsigned int keysize );

/**
 * \brief          AES key schedule (decryption)
 *
 * \param ctx      AES context to be initialized
 * \param key      decryption key
 * \param keysize  must be 128, 192 or 256
 *
 * \return         0 if successful, or ERROR_AES_INVALID_KEY_LENGTH
 */
int AesSetkeyDec( AesContext *ctx, const unsigned char *key, unsigned int keysize );

/**
 * \brief          AES-ECB block encryption/decryption
 *
 * \param ctx      AES context
 * \param mode     AES_ENCRYPT or AES_DECRYPT
 * \param input    16-byte input block
 * \param output   16-byte output block
 *
 * \return         0 if successful
 */
int AesCryptECB( AesContext *ctx,
                int mode,
                const unsigned char input[16],
                unsigned char output[16] );

/**
 * \brief          AES-CBC buffer encryption/decryption
 *                 Length should be a multiple of the block
 *                 size (16 bytes)
 *
 * \param ctx      AES context
 * \param mode     AES_ENCRYPT or AES_DECRYPT
 * \param length   length of the input data
 * \param iv       initialization vector (updated after use)
 * \param input    buffer holding the input data
 * \param output   buffer holding the output data
 *
 * \return         0 if successful, or ERROR_AES_INVALID_INPUT_LENGTH
 */
int AesCryptCBC( AesContext *ctx,
                int mode,
                unsigned int length,
                const unsigned char iv[16],
                const unsigned char *input,
                unsigned char *output );

#ifdef  __cplusplus
}
#endif

#endif /* __AES_H__ */
