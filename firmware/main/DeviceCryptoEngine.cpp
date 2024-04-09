/***********************************************************************************************************************
*                                                                                                                      *
* trigger-crossbar                                                                                                     *
*                                                                                                                      *
* Copyright (c) 2023-2024 Andrew D. Zonenberg and contributors                                                         *
* All rights reserved.                                                                                                 *
*                                                                                                                      *
* Redistribution and use in source and binary forms, with or without modification, are permitted provided that the     *
* following conditions are met:                                                                                        *
*                                                                                                                      *
*    * Redistributions of source code must retain the above copyright notice, this list of conditions, and the         *
*      following disclaimer.                                                                                           *
*                                                                                                                      *
*    * Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the       *
*      following disclaimer in the documentation and/or other materials provided with the distribution.                *
*                                                                                                                      *
*    * Neither the name of the author nor the names of any contributors may be used to endorse or promote products     *
*      derived from this software without specific prior written permission.                                           *
*                                                                                                                      *
* THIS SOFTWARE IS PROVIDED BY THE AUTHORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED   *
* TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL *
* THE AUTHORS BE HELD LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES        *
* (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR       *
* BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT *
* (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE       *
* POSSIBILITY OF SUCH DAMAGE.                                                                                          *
*                                                                                                                      *
***********************************************************************************************************************/

#include "triggercrossbar.h"
#include "DeviceCryptoEngine.h"
#include "../../staticnet/contrib/tweetnacl_25519.h"

DeviceCryptoEngine::DeviceCryptoEngine()
{
}

DeviceCryptoEngine::~DeviceCryptoEngine()
{
}

void DeviceCryptoEngine::SharedSecret(uint8_t* sharedSecret, uint8_t* clientPublicKey)
{
	auto t1 = g_logTimer->GetCount();

	g_fpga->BlockingWrite(REG_CRYPT_BASE + REG_WORK, clientPublicKey, ECDH_KEY_SIZE);
	g_fpga->BlockingWrite(REG_CRYPT_BASE + REG_E, m_ephemeralkeyPriv, ECDH_KEY_SIZE);
	uint8_t status = 1;
	while(status != 0)
		status = g_fpga->BlockingRead8(REG_CRYPT_BASE + REG_CRYPT_STATUS);
	g_fpga->BlockingRead(REG_CRYPT_BASE + REG_WORK_OUT, sharedSecret, ECDH_KEY_SIZE);

	auto delta = g_logTimer->GetCount() - t1;
	g_log("DeviceCryptoEngine::SharedSecret (FPGA acceleration): %d.%d ms\n", delta/10, delta%10);

	//Dump output states
	/*
	g_cliUART->Printf("    clientPublicKey:    ");
	for(uint32_t i=0; i<ECDH_KEY_SIZE; i++)
		g_cliUART->Printf("%02x ", clientPublicKey[i]);
	g_cliUART->Printf("\n");

	g_cliUART->Printf("    m_ephemeralkeyPriv: ");
	for(uint32_t i=0; i<ECDH_KEY_SIZE; i++)
		g_cliUART->Printf("%02x ", m_ephemeralkeyPriv[i]);
	g_cliUART->Printf("\n");

	g_cliUART->Printf("   sharedSecret:        ");
	for(uint32_t i=0; i<ECDH_KEY_SIZE; i++)
		g_cliUART->Printf("%02x ", sharedSecret[i]);
	g_cliUART->Printf("\n");
	*/
}

/**
	@brief Generates an x25519 key pair.

	The private key is kept internal to the CryptoEngine object.

	The public key is stored in the provided buffer, which must be at least 32 bytes in size.
 */
void DeviceCryptoEngine::GenerateX25519KeyPair(uint8_t* pub)
{
	auto t1 = g_logTimer->GetCount();

	//To be a valid key, a few bits need well-defined values. The rest are cryptographic randomness.
	GenerateRandom(m_ephemeralkeyPriv, 32);
	m_ephemeralkeyPriv[0] &= 0xF8;
	m_ephemeralkeyPriv[31] &= 0x7f;
	m_ephemeralkeyPriv[31] |= 0x40;

	//Well defined curve25519 base point from crypto_scalarmult_base
	static uint8_t basepoint[ECDH_KEY_SIZE] =
	{
		9, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
		0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0
	};

	//Make the FPGA do the rest of the work
	g_fpga->BlockingWrite(REG_CRYPT_BASE + REG_WORK, basepoint, ECDH_KEY_SIZE);
	g_fpga->BlockingWrite(REG_CRYPT_BASE + REG_E, m_ephemeralkeyPriv, ECDH_KEY_SIZE);
	uint8_t status = 1;
	while(status != 0)
		status = g_fpga->BlockingRead8(REG_CRYPT_BASE + REG_CRYPT_STATUS);
	g_fpga->BlockingRead(REG_CRYPT_BASE + REG_WORK_OUT, pub, ECDH_KEY_SIZE);

	auto delta = g_logTimer->GetCount() - t1;
	g_log("DeviceCryptoEngine::GenerateX25519KeyPair (FPGA acceleration): %d.%d ms\n", delta/10, delta%10);
}

/**
	@brief Verify a signed message

	The signature is *prepended* to the message: first 64 bytes are signature, then the message
 */
bool DeviceCryptoEngine::VerifySignature(uint8_t* signedMessage, uint32_t lengthIncludingSignature, uint8_t* publicKey)
{
	/*
	//Log the stuff
	g_cliUART->Printf("Signed message:\n");
	for(uint32_t i=0; i<lengthIncludingSignature; i++)
	{
		g_cliUART->Printf("%02x ", signedMessage[i]);
		if((i & 15) == 15)
			g_cliUART->Printf("\n");
	}
	g_cliUART->Printf("\n");

	g_cliUART->Printf("Public key:\n");
	for(uint32_t i=0; i<ECDSA_KEY_SIZE; i++)
	{
		g_cliUART->Printf("%02x ", publicKey[i]);
		if((i & 15) == 15)
			g_cliUART->Printf("\n");
	}
	g_cliUART->Printf("\n");
	*/

	auto t1 = g_logTimer->GetCount();

	//fixed length cap
	if(lengthIncludingSignature > 1024)
		return false;

	//If message isn't big enough to even have a signature it can't be valid
	if (lengthIncludingSignature < ECDSA_SIG_SIZE)
		return false;

	//Hash the input message
	uint8_t hash[SHA512_DIGEST_SIZE];
	unsigned char tmpbuf[1024];
	memcpy(tmpbuf, signedMessage, lengthIncludingSignature);
	memcpy(tmpbuf + ECDSA_KEY_SIZE, publicKey, ECDSA_KEY_SIZE);
	crypto_hash(hash, tmpbuf, lengthIncludingSignature);

	//Modular reduction on the hash to stay within our field
	reduce(hash);

	//Unpack the packed public key
	gf q[4];
	gf p[4];
	if (unpackneg(q, publicKey))
		return false;

	auto t2 = g_logTimer->GetCount();

	//Expanded public key for sending to accelerator
	//TODO: can we move the expansion to the FPGA to save SPI BW and speed compute?
	uint8_t qref[128];
	pack25519(&qref[0], q[0]);
	pack25519(&qref[32], q[1]);
	pack25519(&qref[64], q[2]);
	pack25519(&qref[96], q[3]);

	//Do the first scalarmult() on the FPGA

	//Calculate the expected signature
	//scalarmult(p, q, hash);
	g_fpga->BlockingWrite(REG_CRYPT_BASE + REG_DSA_IN, qref, ECDSA_KEY_SIZE*4);
	g_fpga->BlockingWrite(REG_CRYPT_BASE + REG_E, hash, ECDSA_KEY_SIZE);
	uint8_t status = 1;
	while(status != 0)
		status = g_fpga->BlockingRead8(REG_CRYPT_BASE + REG_CRYPT_STATUS);
	uint8_t pfpga[128];
	for(int block=0; block<4; block++)
	{
		g_fpga->BlockingWrite8(REG_CRYPT_BASE + REG_CRYPT_STATUS, block);
		g_fpga->BlockingRead(REG_CRYPT_BASE + REG_WORK_OUT, pfpga + block*32, ECDH_KEY_SIZE);
	}

	//TEMP: unpack the result and save in p
	unpack25519(p[0], &pfpga[0]);
	unpack25519(p[1], &pfpga[32]);
	unpack25519(p[2], &pfpga[64]);
	unpack25519(p[3], &pfpga[96]);

	auto t3 = g_logTimer->GetCount();

	//scalarbase(q, signedMessage + 32);
	gf gf1 = {1};
	gf X = {0xd51a, 0x8f25, 0x2d60, 0xc956, 0xa7b2, 0x9525, 0xc760, 0x692c, 0xdc5c, 0xfdd6, 0xe231, 0xc0a4, 0x53fe, 0xcd6e, 0x36d3, 0x2169};
	gf Y = {0x6658, 0x6666, 0x6666, 0x6666, 0x6666, 0x6666, 0x6666, 0x6666, 0x6666, 0x6666, 0x6666, 0x6666, 0x6666, 0x6666, 0x6666, 0x6666};
	gf tmp[4];
	set25519(tmp[0],X);
	set25519(tmp[1],Y);
	set25519(tmp[2],gf1);
	M(tmp[3],X,Y);
	scalarmult(q, tmp, signedMessage + 32);

	auto t4 = g_logTimer->GetCount();
	add(p,q);
	uint8_t t[32];
	pack(t,p);

	//Final signature verification
	if (crypto_verify_32(signedMessage, t))
		return false;

	auto tend = g_logTimer->GetCount();
	auto delta = tend - t1;
	g_log("DeviceCryptoEngine::VerifySignature (partial acceleration): %d.%d ms\n", delta/10, delta%10);
	LogIndenter li(g_log);
	delta = t2-t1;
	g_log("Setup (no acceleration): %d.%d ms\n", delta/10, delta%10);
	delta = t3-t2;
	g_log("scalarmult (FPGA acceleration): %d.%d ms\n", delta/10, delta%10);
	delta = t4-t3;
	g_log("scalarbase (no acceleration): %d.%d ms\n", delta/10, delta%10);
	delta = tend-t4;
	g_log("Final (no acceleration): %d.%d ms\n", delta/10, delta%10);

	return true;
}

///@brief Signs an exchange hash with our host key
void DeviceCryptoEngine::SignExchangeHash(uint8_t* sigOut, uint8_t* exchangeHash)
{
	auto t1 = g_logTimer->GetCount();

	//Hash the private key and massage it to make sure it's a valid curve point
	uint8_t privkeyHash[64];
	crypto_hash(privkeyHash, m_hostkeyPriv, 32);
	privkeyHash[0] &= 248;
	privkeyHash[31] &= 127;
	privkeyHash[31] |= 64;

	//Build the actual buffer we're signing
	uint8_t sm[128];
	memcpy(sm+64, exchangeHash, SHA256_DIGEST_SIZE);
	memcpy(sm+32, privkeyHash+32, 32);

	//Hash the buffer and reduce it to make sure it's within our field
	uint8_t bufferHash[64];
	crypto_hash(bufferHash, sm + 32, SHA256_DIGEST_SIZE + 32);
	reduce(bufferHash);

	//Actual signing stuff
	gf p[4];
	scalarbase(p,bufferHash);
	pack(sm,p);

	//Hash the public key
	uint8_t msgHash[64];
	memcpy(sm+32, m_hostkeyPub, ECDSA_KEY_SIZE);
	crypto_hash(msgHash, sm, SHA256_DIGEST_SIZE + 64);
	reduce(msgHash);

	//Bignum stuff on output
	int64_t x[64] = {0};
	for(int i=0; i<32; i++)
		x[i] = (u64) bufferHash[i];
	for(int i=0; i<32; i++)
	{
		for(int j=0; j<32; j++)
			x[i+j] += msgHash[i] * (u64) privkeyHash[j];
	}

	//Final modular reduction and output
	modL(sm + 32,x);
	memcpy(sigOut, sm, 64);

	auto delta = g_logTimer->GetCount() - t1;
	g_log("DeviceCryptoEngine::SignExchangeHash (no acceleration): %d.%d ms\n", delta/10, delta%10);
}
