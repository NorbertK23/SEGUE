/*
 * svc_enc.c
 *
 *  Created on: 11 Mar 2026
 *      Author: Norbert Kania
 */

#ifndef APP_CTRL_DT_MS
#error "APP_CTRL_DT_MS must be defined"
#endif

#if (APP_CTRL_DT_MS == 0)
#error "APP_CTRL_DT_MS must be nonzero"
#endif

#include "avr_stdint.h"

#include "svc_enc.h"
#include "drv_enc.h"

static int32_t countA;
static int32_t countB;

static int32_t prev_countA;
static int32_t prev_countB;

static int32_t deltaA;
static int32_t deltaB;

static int32_t velA_cps;
static int32_t velB_cps;

void svc_enc_init(void)
{
    drv_enc_init();

    countA = drv_enc_get_count_a();
    countB = -drv_enc_get_count_b();

    prev_countA = countA;
    prev_countB = countB;
    deltaA = 0;
    deltaB = 0;
    velA_cps = 0;
    velB_cps = 0;
}

void svc_enc_update(void)
{
    countA = drv_enc_get_count_a();
    countB = -drv_enc_get_count_b();

    deltaA = countA - prev_countA;
    deltaB = countB - prev_countB;

    velA_cps = (deltaA * 1000) / APP_CTRL_DT_MS;
    velB_cps = (deltaB * 1000) / APP_CTRL_DT_MS;

    prev_countA = countA;
    prev_countB = countB;
}



int32_t svc_enc_get_count_a(void)
{
	return countA;
}

int32_t svc_enc_get_count_b(void)
{
	return countB;
}

int32_t svc_enc_get_delta_a(void)
{
	return deltaA;
}

int32_t svc_enc_get_delta_b(void)
{
	return deltaB;
}

int32_t svc_enc_get_vel_a_cps(void)
{
	return velA_cps;
}

int32_t svc_enc_get_vel_b_cps(void)
{
	return velB_cps;
}

void svc_enc_reset(void)
{
    drv_enc_reset();

    countA = drv_enc_get_count_a();
    countB = -drv_enc_get_count_b();

    prev_countA = countA;
    prev_countB = countB;
    deltaA = 0;
    deltaB = 0;
    velA_cps = 0;
    velB_cps = 0;
}

uint32_t svc_enc_get_invalid_a(void)
{
	return drv_enc_get_invalid_a();
}
uint32_t svc_enc_get_invalid_b(void)
{
	return drv_enc_get_invalid_b();
}
