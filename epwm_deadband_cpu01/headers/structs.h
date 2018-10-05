/*
 * structs.h
 *
 *  Created on: 4 de out de 2018
 *      Author: Bruno
 */

#ifndef HEADERS_STRUCTS_H_
#define HEADERS_STRUCTS_H_

//-----------------------------------------------------------------------------/
// Estrutura das vari�veis do conversor
//-----------------------------------------------------------------------------/

    //  ADCA0 -> ia
    //  ADCB0 -> ic
    //  ADCA1 -> vab
    //  ADCB1 -> vcb
    //  ADCA2 -> vccP
    //  ADCB2 -> vccN

struct RETIFICADOR
{
float vdc;
float vab;
float vca;
float vbc;
float ia;
float ib;
float ic;
float id;
float iq;
float m; // Indice de modula��o
float da;               // raz�o c�clica fase a
float db;               // raz�o c�clica fase b
float dc;               // raz�o c�clica fase c
float d0;               // raz�o c�clica eixo zero
float dd;               // raz�o c�clica eixo direto
float dq;               // raz�o c�clica eixo quadratura
float VccRef;           // tens�o de refer�ncia CC
float idref;            // Corrente de refer�ncia de eixo direto
float iqref;            // Corrente de refer�ncia de eixo direto
Uint16 ADCA0;
Uint16 ADCA1;
Uint16 ADCA2;
Uint16 ADCB0;
Uint16 ADCB1;
Uint16 ADCB2;
Uint16 CMPFA; // Valor de compara��o do PWM da fase A
Uint16 CMPFB; // Valor de compara��o do PWM da fase B
Uint16 CMPFC; // Valor de compara��o do PWM da fase C
};


//-----------------------------------------------------------------------------/
// Estrutura de controle do conversor
//-----------------------------------------------------------------------------/

struct CONTROLE
{
char ESTAGIO;
char RELE;
char MF;
char INIT;
char ENABLE;
char STATE;
int AD_EOC;
char PWM;
// float TIME;
// double t;
};




//-----------------------------------------------------------------------------/
// Estrutura para gera��o de sinais trigonom�tricos
//-----------------------------------------------------------------------------/

struct TRIGONOMETRICO
{
float w;
float wt;
float senoA;
float coseA;
};

//-----------------------------------------------------------------------------/
// Estrutura do PLL
//-----------------------------------------------------------------------------/

struct PLL
{
float valfa;
float vbeta;
float ialfa;
float ibeta;
float mod;
float e0;
float e1;
float u1;
float w;
float wt;
float wtx;
int sector;
float seq;      // vari�vel que cont�m sinal para invers�o de sequ�ncia de fase
char ctrl;      // vari�vel com campos booleanos para guardar status de testes
};


//--------------------------------------------------------/
// PI Controller
//--------------------------------------------------------/
struct PI
{
float e0; // Erro atual
float e1; // Erro anterior
float u0; // Sa�da atual
float u1; // Sa�da anterior
};



#endif /* HEADERS_STRUCTS_H_ */
