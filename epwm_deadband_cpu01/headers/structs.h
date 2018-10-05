/*
 * structs.h
 *
 *  Created on: 4 de out de 2018
 *      Author: Bruno
 */

#ifndef HEADERS_STRUCTS_H_
#define HEADERS_STRUCTS_H_

//-----------------------------------------------------------------------------/
// Estrutura das variáveis do conversor
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
float m; // Indice de modulação
float da;               // razão cíclica fase a
float db;               // razão cíclica fase b
float dc;               // razão cíclica fase c
float d0;               // razão cíclica eixo zero
float dd;               // razão cíclica eixo direto
float dq;               // razão cíclica eixo quadratura
float VccRef;           // tensão de referência CC
float idref;            // Corrente de referência de eixo direto
float iqref;            // Corrente de referência de eixo direto
Uint16 ADCA0;
Uint16 ADCA1;
Uint16 ADCA2;
Uint16 ADCB0;
Uint16 ADCB1;
Uint16 ADCB2;
Uint16 CMPFA; // Valor de comparação do PWM da fase A
Uint16 CMPFB; // Valor de comparação do PWM da fase B
Uint16 CMPFC; // Valor de comparação do PWM da fase C
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
// Estrutura para geração de sinais trigonométricos
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
float seq;      // variável que contém sinal para inversão de sequência de fase
char ctrl;      // variável com campos booleanos para guardar status de testes
};


//--------------------------------------------------------/
// PI Controller
//--------------------------------------------------------/
struct PI
{
float e0; // Erro atual
float e1; // Erro anterior
float u0; // Saída atual
float u1; // Saída anterior
};



#endif /* HEADERS_STRUCTS_H_ */
