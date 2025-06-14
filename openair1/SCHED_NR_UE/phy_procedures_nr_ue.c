/*
 * Licensed to the OpenAirInterface (OAI) Software Alliance under one or more
 * contributor license agreements.  See the NOTICE file distributed with
 * this work for additional information regarding copyright ownership.
 * The OpenAirInterface Software Alliance licenses this file to You under
 * the OAI Public License, Version 1.1  (the "License"); you may not use this file
 * except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.openairinterface.org/?page_id=698
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *-------------------------------------------------------------------------------
 * For more information about the OpenAirInterface (OAI) Software Alliance:
 *      contact@openairinterface.org
 */

/*! \file phy_procedures_nr_ue.c
 * \brief Implementation of UE procedures from 36.213 LTE specifications
 * \author R. Knopp, F. Kaltenberger, N. Nikaein, A. Mico Pereperez, G. Casati
 * \date 2018
 * \version 0.1
 * \company Eurecom
 * \email: knopp@eurecom.fr,florian.kaltenberger@eurecom.fr, navid.nikaein@eurecom.fr, guido.casati@iis.fraunhofer.de
 * \note
 * \warning
 */

#define _GNU_SOURCE

#include "nr/nr_common.h"
#include "assertions.h"
#include "defs.h"
#include "PHY/defs_nr_UE.h"
#include "PHY/NR_REFSIG/dmrs_nr.h"
#include "PHY/phy_extern_nr_ue.h"
#include "PHY/MODULATION/modulation_UE.h"
#include "PHY/INIT/phy_init.h"
#include "PHY/NR_UE_TRANSPORT/nr_transport_ue.h"
#include "PHY/NR_UE_TRANSPORT/nr_transport_proto_ue.h"
#include "PHY/NR_UE_TRANSPORT/srs_modulation_nr.h"
#include "SCHED_NR/extern.h"
#include "SCHED_NR_UE/phy_sch_processing_time.h"
#include "PHY/NR_UE_ESTIMATION/nr_estimation.h"
#ifdef EMOS
#include "SCHED/phy_procedures_emos.h"
#endif
#include "executables/softmodem-common.h"
#include "executables/nr-uesoftmodem.h"
#include "LAYER2/NR_MAC_UE/mac_proto.h"
#include "LAYER2/NR_MAC_UE/nr_l1_helpers.h"
#include "openair1/PHY/MODULATION/nr_modulation.h"
#include "openair1/SIMULATION/TOOLS/sim.h"
//#define DEBUG_PHY_PROC
#include <sys/socket.h>
#include <fcntl.h>
#include <arpa/inet.h>

#define NR_PDCCH_SCHED
//#define NR_PDCCH_SCHED_DEBUG
//#define NR_PUCCH_SCHED
//#define NR_PUCCH_SCHED_DEBUG
//#define NR_PDSCH_DEBUG
#define HNA_SIZE 16 * 68 * 384 // [hna] 16 segments, 68*Zc
#define LDPC_MAX_LIMIT 63
#ifndef PUCCH
#define PUCCH
#endif


#include "common/utils/LOG/log.h"

#ifdef EMOS
fifo_dump_emos_UE emos_dump_UE;
#endif

#include "common/utils/LOG/vcd_signal_dumper.h"
#include "UTIL/OPT/opt.h"
#include "intertask_interface.h"
#include "T.h"

char nr_mode_string[NUM_UE_MODE][20] = {"NOT SYNCHED","PRACH","RAR","RA_WAIT_CR", "PUSCH", "RESYNCH"};

#if defined(EXMIMO) || defined(OAI_USRP) || defined(OAI_BLADERF) || defined(OAI_LMSSDR) || defined(OAI_ADRV9371_ZC706)
extern uint64_t downlink_frequency[MAX_NUM_CCs][4];
#endif

unsigned int gain_table[31] = {100,112,126,141,158,178,200,224,251,282,316,359,398,447,501,562,631,708,794,891,1000,1122,1258,1412,1585,1778,1995,2239,2512,2818,3162};

void nr_fill_dl_indication(nr_downlink_indication_t *dl_ind,
                           fapi_nr_dci_indication_t *dci_ind,
                           fapi_nr_rx_indication_t *rx_ind,
                           UE_nr_rxtx_proc_t *proc,
                           PHY_VARS_NR_UE *ue,
                           uint8_t gNB_id,
                           void *phy_data){

  memset((void*)dl_ind, 0, sizeof(nr_downlink_indication_t));

  dl_ind->gNB_index = gNB_id;
  dl_ind->module_id = ue->Mod_id;
  dl_ind->cc_id     = ue->CC_id;
  dl_ind->frame     = proc->frame_rx;
  dl_ind->slot      = proc->nr_slot_rx;
  dl_ind->thread_id = proc->thread_id;
  dl_ind->phy_data  = phy_data;

  if (dci_ind) {

    dl_ind->rx_ind = NULL; //no data, only dci for now
    dl_ind->dci_ind = dci_ind;

  } else if (rx_ind) {

    dl_ind->rx_ind = rx_ind; //  hang on rx_ind instance
    dl_ind->dci_ind = NULL;

  }
}

void nr_fill_rx_indication(fapi_nr_rx_indication_t *rx_ind,
                           uint8_t pdu_type,
                           uint8_t gNB_id,
                           PHY_VARS_NR_UE *ue,
                           NR_UE_DLSCH_t *dlsch0,
                           NR_UE_DLSCH_t *dlsch1,
                           uint16_t n_pdus,
                           UE_nr_rxtx_proc_t *proc,
                           void *typeSpecific){

  NR_DL_FRAME_PARMS *frame_parms = &ue->frame_parms;

  if (n_pdus > 1){
    LOG_E(PHY, "In %s: multiple number of DL PDUs not supported yet...\n", __FUNCTION__);
  }

  if (pdu_type !=  FAPI_NR_RX_PDU_TYPE_SSB)
    trace_NRpdu(DIRECTION_DOWNLINK,
		dlsch0->harq_processes[dlsch0->current_harq_pid]->b,
		dlsch0->harq_processes[dlsch0->current_harq_pid]->TBS / 8,
		WS_C_RNTI,
		dlsch0->rnti,
		proc->frame_rx,
		proc->nr_slot_rx,
		0,0);
  switch (pdu_type){
    case FAPI_NR_RX_PDU_TYPE_SIB:
      rx_ind->rx_indication_body[n_pdus - 1].pdsch_pdu.harq_pid = dlsch0->current_harq_pid;
      rx_ind->rx_indication_body[n_pdus - 1].pdsch_pdu.ack_nack = dlsch0->harq_processes[dlsch0->current_harq_pid]->ack;
      rx_ind->rx_indication_body[n_pdus - 1].pdsch_pdu.pdu = dlsch0->harq_processes[dlsch0->current_harq_pid]->b;
      rx_ind->rx_indication_body[n_pdus - 1].pdsch_pdu.pdu_length = dlsch0->harq_processes[dlsch0->current_harq_pid]->TBS / 8;
    break;
    case FAPI_NR_RX_PDU_TYPE_DLSCH:
      if(dlsch0) {
        rx_ind->rx_indication_body[n_pdus - 1].pdsch_pdu.harq_pid = dlsch0->current_harq_pid;
        rx_ind->rx_indication_body[n_pdus - 1].pdsch_pdu.ack_nack = dlsch0->harq_processes[dlsch0->current_harq_pid]->ack;
        rx_ind->rx_indication_body[n_pdus - 1].pdsch_pdu.pdu = dlsch0->harq_processes[dlsch0->current_harq_pid]->b;
        rx_ind->rx_indication_body[n_pdus - 1].pdsch_pdu.pdu_length = dlsch0->harq_processes[dlsch0->current_harq_pid]->TBS / 8;
      }
      if(dlsch1) {
        AssertFatal(1==0,"Second codeword currently not supported\n");
      }
      break;
    case FAPI_NR_RX_PDU_TYPE_RAR:
      rx_ind->rx_indication_body[n_pdus - 1].pdsch_pdu.harq_pid = dlsch0->current_harq_pid;
      rx_ind->rx_indication_body[n_pdus - 1].pdsch_pdu.ack_nack = dlsch0->harq_processes[dlsch0->current_harq_pid]->ack;
      rx_ind->rx_indication_body[n_pdus - 1].pdsch_pdu.pdu = dlsch0->harq_processes[dlsch0->current_harq_pid]->b;
      rx_ind->rx_indication_body[n_pdus - 1].pdsch_pdu.pdu_length = dlsch0->harq_processes[dlsch0->current_harq_pid]->TBS / 8;
    break;
    case FAPI_NR_RX_PDU_TYPE_SSB:
      rx_ind->rx_indication_body[n_pdus - 1].ssb_pdu.pdu=malloc(sizeof(((fapiPbch_t*)typeSpecific)->decoded_output));
      memcpy(rx_ind->rx_indication_body[n_pdus - 1].ssb_pdu.pdu,
	     ((fapiPbch_t*)typeSpecific)->decoded_output,
	     sizeof(((fapiPbch_t*)typeSpecific)->decoded_output));
      rx_ind->rx_indication_body[n_pdus - 1].ssb_pdu.additional_bits = ((fapiPbch_t*)typeSpecific)->xtra_byte;
      rx_ind->rx_indication_body[n_pdus - 1].ssb_pdu.ssb_index = (frame_parms->ssb_index)&0x7;
      rx_ind->rx_indication_body[n_pdus - 1].ssb_pdu.ssb_length = frame_parms->Lmax;
      rx_ind->rx_indication_body[n_pdus - 1].ssb_pdu.cell_id = frame_parms->Nid_cell;
      rx_ind->rx_indication_body[n_pdus - 1].ssb_pdu.ssb_start_subcarrier = frame_parms->ssb_start_subcarrier;
      rx_ind->rx_indication_body[n_pdus - 1].ssb_pdu.rsrp_dBm = ue->measurements.ssb_rsrp_dBm[frame_parms->ssb_index];
    break;
    case FAPI_NR_CSIRS_IND:
      memcpy(&rx_ind->rx_indication_body[n_pdus - 1].csirs_measurements,
             (fapi_nr_csirs_measurements_t*)typeSpecific,
             sizeof(*(fapi_nr_csirs_measurements_t*)typeSpecific));
      break;
    default:
    break;
  }

  rx_ind->rx_indication_body[n_pdus -1].pdu_type = pdu_type;
  rx_ind->number_pdus = n_pdus;

}

int get_tx_amp_prach(int power_dBm, int power_max_dBm, int N_RB_UL){

  int gain_dB = power_dBm - power_max_dBm, amp_x_100 = -1;

  switch (N_RB_UL) {
  case 6:
  amp_x_100 = AMP;      // PRACH is 6 PRBS so no scale
  break;
  case 15:
  amp_x_100 = 158*AMP;  // 158 = 100*sqrt(15/6)
  break;
  case 25:
  amp_x_100 = 204*AMP;  // 204 = 100*sqrt(25/6)
  break;
  case 50:
  amp_x_100 = 286*AMP;  // 286 = 100*sqrt(50/6)
  break;
  case 75:
  amp_x_100 = 354*AMP;  // 354 = 100*sqrt(75/6)
  break;
  case 100:
  amp_x_100 = 408*AMP;  // 408 = 100*sqrt(100/6)
  break;
  default:
  LOG_E(PHY, "Unknown PRB size %d\n", N_RB_UL);
  return (amp_x_100);
  break;
  }
  if (gain_dB < -30) {
    return (amp_x_100/3162);
  } else if (gain_dB > 0)
    return (amp_x_100);
  else
    return (amp_x_100/gain_table[-gain_dB]);  // 245 corresponds to the factor sqrt(25/6)

  return (amp_x_100);
}

UE_MODE_t get_nrUE_mode(uint8_t Mod_id,uint8_t CC_id,uint8_t gNB_id){
  return(PHY_vars_UE_g[Mod_id][CC_id]->UE_mode[gNB_id]);
}

// convert time factor "16 * 64 * T_c / (2^mu)" in N_TA calculation in TS38.213 section 4.2 to samples by multiplying with samples per second
//   16 * 64 * T_c            / (2^mu) * samples_per_second
// = 16 * T_s                 / (2^mu) * samples_per_second
// = 16 * 1 / (15 kHz * 2048) / (2^mu) * (15 kHz * 2^mu * ofdm_symbol_size)
// = 16 * 1 /           2048           *                  ofdm_symbol_size
// = 16 * ofdm_symbol_size / 2048
static inline
uint16_t get_bw_scaling(uint16_t ofdm_symbol_size){
  return 16 * ofdm_symbol_size / 2048;
}

// UL time alignment procedures:
// - If the current tx frame and slot match the TA configuration in ul_time_alignment
//   then timing advance is processed and set to be applied in the next UL transmission
// - Application of timing adjustment according to TS 38.213 p4.2
// todo:
// - handle RAR TA application as per ch 4.2 TS 38.213
void ue_ta_procedures(PHY_VARS_NR_UE *ue, int slot_tx, int frame_tx){

  if (ue->mac_enabled == 1) {

    uint8_t gNB_id = 0;
    NR_UL_TIME_ALIGNMENT_t *ul_time_alignment = &ue->ul_time_alignment[gNB_id];

    if (frame_tx == ul_time_alignment->ta_frame && slot_tx == ul_time_alignment->ta_slot) {

      uint16_t ofdm_symbol_size = ue->frame_parms.ofdm_symbol_size;
      uint16_t bw_scaling = get_bw_scaling(ofdm_symbol_size);

      ue->timing_advance += (ul_time_alignment->ta_command - 31) * bw_scaling;

      LOG_D(PHY, "In %s: [UE %d] [%d.%d] Got timing advance command %u from MAC, new value is %d\n",
        __FUNCTION__,
        ue->Mod_id,
        frame_tx,
        slot_tx,
        ul_time_alignment->ta_command,
        ue->timing_advance);

      ul_time_alignment->ta_frame = -1;
      ul_time_alignment->ta_slot = -1;

    }
  }
}

bool phy_ssb_slot_allocation_sl(PHY_VARS_NR_UE *ue, int frame, int slot)
{
  NR_DL_FRAME_PARMS *fp = &ue->frame_parms;
  static int sl_numssb_withinperiod_r16, sl_timeoffsetssb_r16, sl_timeoffsetssb_r16_copy;

  if (sl_numssb_withinperiod_r16 == 0) {
    sl_numssb_withinperiod_r16 = ue->slss->sl_numssb_withinperiod_r16;
    sl_timeoffsetssb_r16 = ue->slss->sl_timeoffsetssb_r16;
    sl_timeoffsetssb_r16_copy = sl_timeoffsetssb_r16;
  }

  if ((frame * fp->slots_per_frame + slot) % (16 * fp->slots_per_frame) == 0) {
    ue->slss->sl_numssb_withinperiod_r16 = ue->slss->sl_numssb_withinperiod_r16_copy;
    sl_timeoffsetssb_r16 = frame * fp->slots_per_frame + sl_timeoffsetssb_r16_copy;
  }

  if (ue->slss->sl_numssb_withinperiod_r16 > 0) {
    if (frame * fp->slots_per_frame + slot == sl_timeoffsetssb_r16) {
      sl_timeoffsetssb_r16 = sl_timeoffsetssb_r16 + ue->slss->sl_timeinterval_r16;
      ue->slss->sl_numssb_withinperiod_r16 = ue->slss->sl_numssb_withinperiod_r16 - 1;
      LOG_I(PHY,"*** SL-SSB slot allocation  %d.%d ***\n", frame, slot); 
    } else {
      return false;
    }
  } else {
    return false;
  }
  return true;
}

void phy_procedures_nrUE_SL_TX(PHY_VARS_NR_UE *ue,
                               UE_nr_rxtx_proc_t *proc,
                               uint8_t gNB_id)
{
  int slot_tx = proc->nr_slot_tx;
  int frame_tx = proc->frame_tx;
  AssertFatal(frame_tx >= 0 && frame_tx < 1024, "frame_tx %d is not in 0...1023\n",frame_tx);
  AssertFatal(slot_tx >= 0 && slot_tx < 20, "slot_tx %d is not in 0...19\n", slot_tx);

  if (get_softmodem_params()->sl_mode == 2) {
    ue->tx_power_dBm[slot_tx] = -127;
    int num_samples_per_slot = ue->frame_parms.slots_per_frame * ue->frame_parms.samples_per_slot_wCP;
    for(int i = 0; i < ue->frame_parms.nb_antennas_tx; ++i) {
      AssertFatal(i < sizeof(ue->common_vars.txdataF), "Array index %d is over the Array size %lu\n", i, sizeof(ue->common_vars.txdataF));
      memset(ue->common_vars.txdataF[i], 0, sizeof(int32_t) * num_samples_per_slot);
    }
  }

  if (ue->sync_ref && phy_ssb_slot_allocation_sl(ue, frame_tx, slot_tx)) {
    nr_sl_common_signal_procedures(ue, frame_tx, slot_tx);
    const int txdataF_offset = slot_tx * ue->frame_parms.samples_per_slot_wCP;
    LOG_D(NR_PHY, "%s() %d. slot %d txdataF_offset %d\n", __FUNCTION__, __LINE__, slot_tx, txdataF_offset);
    uint16_t nb_prefix_samples0 = ue->is_synchronized_sl ? ue->frame_parms.nb_prefix_samples0 : ue->frame_parms.nb_prefix_samples;
    int slot_timestamp = ue->frame_parms.get_samples_slot_timestamp(slot_tx, &ue->frame_parms, 0);
    for (int aa = 0; aa < ue->frame_parms.nb_antennas_tx; aa++) {
      apply_nr_rotation(&ue->frame_parms,
                        (int16_t*)&ue->common_vars.txdataF[aa][txdataF_offset],
                        slot_tx, 0, 1, link_type_sl); // Conducts rotation on 0th symbol
      PHY_ofdm_mod(&ue->common_vars.txdataF[aa][txdataF_offset],
                    (int*)&ue->common_vars.txdata[aa][slot_timestamp],
                    ue->frame_parms.ofdm_symbol_size,
                    1, // Takes IDFT of 1st symbol (first PSBCH)
                    ue->frame_parms.nb_prefix_samples0,
                    CYCLIC_PREFIX);
      apply_nr_rotation(&ue->frame_parms,
                        (int16_t*)&ue->common_vars.txdataF[aa][txdataF_offset],
                       slot_tx, 1, 13, link_type_sl); // Conducts rotation on symbols located 1 (PSS) to 13 (guard)
      PHY_ofdm_mod(&ue->common_vars.txdataF[aa][ue->frame_parms.ofdm_symbol_size + txdataF_offset], // Starting at PSS (in freq)
                    (int*)&ue->common_vars.txdata[aa][ue->frame_parms.ofdm_symbol_size +
                                      nb_prefix_samples0 +
                                      ue->frame_parms.nb_prefix_samples +
                                      slot_timestamp], // Starting output offset at CP0 + PSBCH0 + CP1
                    ue->frame_parms.ofdm_symbol_size,
                    13, // Takes IDFT of remaining 13 symbols (PSS to guard)... Notice the offset of the input and output above
                    ue->frame_parms.nb_prefix_samples,
                    CYCLIC_PREFIX);
    }
#ifdef DEBUG_PHY_PROC
    char buffer1[ue->frame_parms.ofdm_symbol_size];
    for (int i = 0; i < 13; i++) {
      bzero(buffer1, sizeof(buffer1));
      LOG_I(NR_PHY, "%s(): %d After rotation txdataF[%d] = %s\n",
           __FUNCTION__, __LINE__,  txdataF_offset + (ue->frame_parms.ofdm_symbol_size * i),
           hexdump((void *)&ue->common_vars.txdataF[0][txdataF_offset + (ue->frame_parms.ofdm_symbol_size * i)], ue->frame_parms.ofdm_symbol_size, buffer1, sizeof(buffer1)));
    }
    char buffer0[ue->frame_parms.ofdm_symbol_size];
    for (int i = 0; i < 13; i++) {
      bzero(buffer0, sizeof(buffer0));
      LOG_I(NR_PHY, "%s(): %d Time domain txdata[%d] = %s\n",
           __FUNCTION__, __LINE__,  slot_timestamp + ue->frame_parms.ofdm_symbol_size * i,
           hexdump((void *)&ue->common_vars.txdata[0][slot_timestamp + ue->frame_parms.ofdm_symbol_size * i], ue->frame_parms.ofdm_symbol_size, buffer0, sizeof(buffer0)));
    }
#endif
  } else {
    uint32_t sl_bitmap_tx = 0x00000;
    if(ue->sync_ref) {
      sl_bitmap_tx = (ue->is_synchronized_sl == 0) ? 0x00001 : 0x00002;  // SyncRef UE tx slot 0, Relay UE B tx slot 1.
    }
    if ((sl_bitmap_tx >> slot_tx) & 1) {
      for (uint8_t harq_pid = 0; harq_pid < 1; harq_pid++) {
        nr_ue_set_slsch(&ue->frame_parms, harq_pid, ue->slsch[proc->thread_id][gNB_id], frame_tx, slot_tx);
        if (ue->slsch[proc->thread_id][gNB_id]->harq_processes[harq_pid]->status == ACTIVE) {
          nr_ue_slsch_tx_procedures(ue, harq_pid, frame_tx, slot_tx);
        }
      }
    }
  }
  LOG_D(PHY,"****** end Sidelink TX-Chain for AbsSlot %d.%d ******\n", frame_tx, slot_tx);
}

void phy_procedures_nrUE_TX(PHY_VARS_NR_UE *ue,
                            UE_nr_rxtx_proc_t *proc,
                            uint8_t gNB_id) {

  int slot_tx = proc->nr_slot_tx;
  int frame_tx = proc->frame_tx;

  AssertFatal(ue->CC_id == 0, "Transmission on secondary CCs is not supported yet\n");

  VCD_SIGNAL_DUMPER_DUMP_FUNCTION_BY_NAME(VCD_SIGNAL_DUMPER_FUNCTIONS_PHY_PROCEDURES_UE_TX,VCD_FUNCTION_IN);

  for(int i=0; i< ue->frame_parms.nb_antennas_tx; ++i)
    memset(ue->common_vars.txdataF[i], 0, sizeof(int)*14*ue->frame_parms.ofdm_symbol_size);

  LOG_D(PHY,"****** start TX-Chain for AbsSubframe %d.%d ******\n", frame_tx, slot_tx);

  start_meas(&ue->phy_proc_tx);

  if (ue->UE_mode[gNB_id] <= PUSCH){

    for (uint8_t harq_pid = 0; harq_pid < ue->ulsch[proc->thread_id][gNB_id]->number_harq_processes_for_pusch; harq_pid++) {
      if (ue->ulsch[proc->thread_id][gNB_id]->harq_processes[harq_pid]->status == ACTIVE)
        nr_ue_ulsch_procedures(ue, harq_pid, frame_tx, slot_tx, proc->thread_id, gNB_id);
    }
  }

  if (ue->UE_mode[gNB_id] == PUSCH) {
    ue_srs_procedures_nr(ue, proc, gNB_id);
  }

  VCD_SIGNAL_DUMPER_DUMP_FUNCTION_BY_NAME(VCD_SIGNAL_DUMPER_FUNCTIONS_PHY_PROCEDURES_UE_TX, VCD_FUNCTION_OUT);
  stop_meas(&ue->phy_proc_tx);


}

void nr_ue_measurement_procedures(uint16_t l,
                                  PHY_VARS_NR_UE *ue,
                                  UE_nr_rxtx_proc_t *proc,
                                  uint8_t gNB_id,
                                  uint16_t slot){

  NR_DL_FRAME_PARMS *frame_parms=&ue->frame_parms;
  int frame_rx   = proc->frame_rx;
  int nr_slot_rx = proc->nr_slot_rx;
  VCD_SIGNAL_DUMPER_DUMP_FUNCTION_BY_NAME(VCD_SIGNAL_DUMPER_FUNCTIONS_UE_MEASUREMENT_PROCEDURES, VCD_FUNCTION_IN);

  if (l==2) {

    LOG_D(PHY,"Doing UE measurement procedures in symbol l %u Ncp %d nr_slot_rx %d, rxdata %p\n",
      l,
      ue->frame_parms.Ncp,
      nr_slot_rx,
      ue->common_vars.rxdata);

    nr_ue_measurements(ue, proc, nr_slot_rx);

#if T_TRACER
    if(slot == 0)
      T(T_UE_PHY_MEAS, T_INT(gNB_id),  T_INT(ue->Mod_id), T_INT(frame_rx%1024), T_INT(nr_slot_rx),
	T_INT((int)(10*log10(ue->measurements.rsrp[0])-ue->rx_total_gain_dB)),
	T_INT((int)ue->measurements.rx_rssi_dBm[0]),
	T_INT((int)(ue->measurements.rx_power_avg_dB[0] - ue->measurements.n0_power_avg_dB)),
	T_INT((int)ue->measurements.rx_power_avg_dB[0]),
	T_INT((int)ue->measurements.n0_power_avg_dB),
	T_INT((int)ue->measurements.wideband_cqi_avg[0]),
	T_INT((int)ue->common_vars.freq_offset));
#endif
  }

  // accumulate and filter timing offset estimation every subframe (instead of every frame)
  if (( slot == 2) && (l==(2-frame_parms->Ncp))) {

    // AGC

    VCD_SIGNAL_DUMPER_DUMP_FUNCTION_BY_NAME(VCD_SIGNAL_DUMPER_FUNCTIONS_UE_GAIN_CONTROL, VCD_FUNCTION_IN);


    //printf("start adjust gain power avg db %d\n", ue->measurements.rx_power_avg_dB[gNB_id]);
    phy_adjust_gain_nr (ue,ue->measurements.rx_power_avg_dB[gNB_id],gNB_id);
    
    VCD_SIGNAL_DUMPER_DUMP_FUNCTION_BY_NAME(VCD_SIGNAL_DUMPER_FUNCTIONS_UE_GAIN_CONTROL, VCD_FUNCTION_OUT);

}

  VCD_SIGNAL_DUMPER_DUMP_FUNCTION_BY_NAME(VCD_SIGNAL_DUMPER_FUNCTIONS_UE_MEASUREMENT_PROCEDURES, VCD_FUNCTION_OUT);
}

static void nr_ue_pbch_procedures(uint8_t gNB_id,
                                  PHY_VARS_NR_UE *ue,
                                  UE_nr_rxtx_proc_t *proc,
                                  int estimateSz,
                                  struct complex16 dl_ch_estimates[][estimateSz],
                                  NR_UE_PDCCH_CONFIG *phy_pdcch_config) {

  int ret = 0;
  DevAssert(ue);

  int frame_rx = proc->frame_rx;
  int nr_slot_rx = proc->nr_slot_rx;

  VCD_SIGNAL_DUMPER_DUMP_FUNCTION_BY_NAME(VCD_SIGNAL_DUMPER_FUNCTIONS_UE_PBCH_PROCEDURES, VCD_FUNCTION_IN);

  LOG_D(PHY,"[UE  %d] Frame %d Slot %d, Trying PBCH (NidCell %d, gNB_id %d)\n",ue->Mod_id,frame_rx,nr_slot_rx,ue->frame_parms.Nid_cell,gNB_id);
  fapiPbch_t result;
  ret = nr_rx_pbch(ue, proc,
		   estimateSz, dl_ch_estimates,
                   ue->pbch_vars[gNB_id],
                   &ue->frame_parms,
                   gNB_id,
                   (ue->frame_parms.ssb_index)&7,
                   SISO,
                   phy_pdcch_config,
                   &result);

  if (ret==0) {

    ue->pbch_vars[gNB_id]->pdu_errors_conseq = 0;

    // Switch to PRACH state if it is first PBCH after initial synch and no timing correction is performed
    if (ue->UE_mode[gNB_id] == NOT_SYNCHED && ue->no_timing_correction == 1){
      if (get_softmodem_params()->do_ra) {
        ue->UE_mode[gNB_id] = PRACH;
        ue->prach_resources[gNB_id]->sync_frame = frame_rx;
        ue->prach_resources[gNB_id]->init_msg1 = 0;
      } else {
        ue->UE_mode[gNB_id] = PUSCH;
      }
    }

#ifdef DEBUG_PHY_PROC
    uint16_t frame_tx;
    LOG_D(PHY,"[UE %d] frame %d, nr_slot_rx %d, Received PBCH (MIB): frame_tx %d. N_RB_DL %d\n",
    ue->Mod_id,
    frame_rx,
    nr_slot_rx,
    frame_tx,
    ue->frame_parms.N_RB_DL);
#endif

  } else {
    LOG_E(PHY,"[UE %d] frame %d, nr_slot_rx %d, Error decoding PBCH!\n",
	  ue->Mod_id,frame_rx, nr_slot_rx);
    /*FILE *fd;
    if ((fd = fopen("rxsig_frame0.dat","w")) != NULL) {
                  fwrite((void *)&ue->common_vars.rxdata[0][0],
                         sizeof(int32_t),
                         ue->frame_parms.samples_per_frame,
                         fd);
                  LOG_I(PHY,"Dummping Frame ... bye bye \n");
                  fclose(fd);
                  exit(0);
                }*/

    /*
    write_output("rxsig0.m","rxs0", ue->common_vars.rxdata[0],ue->frame_parms.samples_per_subframe,1,1);


      write_output("H00.m","h00",&(ue->common_vars.dl_ch_estimates[0][0][0]),((ue->frame_parms.Ncp==0)?7:6)*(ue->frame_parms.ofdm_symbol_size),1,1);
      write_output("H10.m","h10",&(ue->common_vars.dl_ch_estimates[0][2][0]),((ue->frame_parms.Ncp==0)?7:6)*(ue->frame_parms.ofdm_symbol_size),1,1);

      write_output("rxsigF0.m","rxsF0", ue->common_vars.rxdataF[0],8*ue->frame_parms.ofdm_symbol_size,1,1);
      write_output("PBCH_rxF0_ext.m","pbch0_ext",ue->pbch_vars[0]->rxdataF_ext[0],12*4*6,1,1);
      write_output("PBCH_rxF0_comp.m","pbch0_comp",ue->pbch_vars[0]->rxdataF_comp[0],12*4*6,1,1);
      write_output("PBCH_rxF_llr.m","pbch_llr",ue->pbch_vars[0]->llr,(ue->frame_parms.Ncp==0) ? 1920 : 1728,1,4);
      exit(-1);
    */

    ue->pbch_vars[gNB_id]->pdu_errors_conseq++;
    ue->pbch_vars[gNB_id]->pdu_errors++;

    if (ue->pbch_vars[gNB_id]->pdu_errors_conseq>=100) {
      if (get_softmodem_params()->non_stop) {
        LOG_E(PHY,"More that 100 consecutive PBCH errors! Going back to Sync mode!\n");
        ue->lost_sync = 1;
      } else {
        LOG_E(PHY,"More that 100 consecutive PBCH errors! Exiting!\n");
        exit_fun("More that 100 consecutive PBCH errors! Exiting!\n");
      }
    }
  }

  if (frame_rx % 100 == 0) {
    ue->pbch_vars[gNB_id]->pdu_errors_last = ue->pbch_vars[gNB_id]->pdu_errors;
  }

#ifdef DEBUG_PHY_PROC
  LOG_D(PHY,"[UE %d] frame %d, slot %d, PBCH errors = %d, consecutive errors = %d!\n",
	ue->Mod_id,frame_rx, nr_slot_rx,
	ue->pbch_vars[gNB_id]->pdu_errors,
	ue->pbch_vars[gNB_id]->pdu_errors_conseq);
#endif
  VCD_SIGNAL_DUMPER_DUMP_FUNCTION_BY_NAME(VCD_SIGNAL_DUMPER_FUNCTIONS_UE_PBCH_PROCEDURES, VCD_FUNCTION_OUT);
}



unsigned int nr_get_tx_amp(int power_dBm, int power_max_dBm, int N_RB_UL, int nb_rb)
{

  int gain_dB = power_dBm - power_max_dBm;
  double gain_lin;

  gain_lin = pow(10,.1*gain_dB);
  if ((nb_rb >0) && (nb_rb <= N_RB_UL)) {
    return((int)(AMP*sqrt(gain_lin*N_RB_UL/(double)nb_rb)));
  }
  else {
    LOG_E(PHY,"Illegal nb_rb/N_RB_UL combination (%d/%d)\n",nb_rb,N_RB_UL);
    //mac_xface->macphy_exit("");
  }
  return(0);
}

#ifdef NR_PDCCH_SCHED

int nr_ue_pdcch_procedures(uint8_t gNB_id,
                           PHY_VARS_NR_UE *ue,
                           UE_nr_rxtx_proc_t *proc,
                           int32_t pdcch_est_size,
                           int32_t pdcch_dl_ch_estimates[][pdcch_est_size],
                           NR_UE_PDCCH_CONFIG *phy_pdcch_config,
                           int n_ss)
{
  int frame_rx = proc->frame_rx;
  int nr_slot_rx = proc->nr_slot_rx;
  unsigned int dci_cnt=0;
  fapi_nr_dci_indication_t *dci_ind = calloc(1, sizeof(*dci_ind));
  nr_downlink_indication_t dl_indication;

  fapi_nr_dl_config_dci_dl_pdu_rel15_t *rel15 = &phy_pdcch_config->pdcch_config[n_ss];

  start_meas(&ue->dlsch_rx_pdcch_stats);

  /// PDCCH/DCI e-sequence (input to rate matching).
  int32_t pdcch_e_rx_size = NR_MAX_PDCCH_SIZE;
  int16_t pdcch_e_rx[pdcch_e_rx_size];

  VCD_SIGNAL_DUMPER_DUMP_FUNCTION_BY_NAME(VCD_SIGNAL_DUMPER_FUNCTIONS_RX_PDCCH, VCD_FUNCTION_IN);
  nr_rx_pdcch(ue, proc, pdcch_est_size, pdcch_dl_ch_estimates, pdcch_e_rx, rel15);
  VCD_SIGNAL_DUMPER_DUMP_FUNCTION_BY_NAME(VCD_SIGNAL_DUMPER_FUNCTIONS_RX_PDCCH, VCD_FUNCTION_OUT);
  

  VCD_SIGNAL_DUMPER_DUMP_FUNCTION_BY_NAME(VCD_SIGNAL_DUMPER_FUNCTIONS_DCI_DECODING, VCD_FUNCTION_IN);

#ifdef NR_PDCCH_SCHED_DEBUG
  printf("<-NR_PDCCH_PHY_PROCEDURES_LTE_UE (nr_ue_pdcch_procedures)-> Entering function nr_dci_decoding_procedure for search space %d)\n",
	 n_ss);
#endif

  dci_cnt = nr_dci_decoding_procedure(ue, proc, pdcch_e_rx, dci_ind, rel15, phy_pdcch_config);

#ifdef NR_PDCCH_SCHED_DEBUG
  LOG_I(PHY,"<-NR_PDCCH_PHY_PROCEDURES_LTE_UE (nr_ue_pdcch_procedures)-> Ending function nr_dci_decoding_procedure() -> dci_cnt=%u\n",dci_cnt);
#endif
  
  VCD_SIGNAL_DUMPER_DUMP_FUNCTION_BY_NAME(VCD_SIGNAL_DUMPER_FUNCTIONS_DCI_DECODING, VCD_FUNCTION_OUT);
  //LOG_D(PHY,"[UE  %d][PUSCH] Frame %d nr_slot_rx %d PHICH RX\n",ue->Mod_id,frame_rx,nr_slot_rx);

  for (int i=0; i<dci_cnt; i++) {
    LOG_D(PHY,"[UE  %d] AbsSubFrame %d.%d, Mode %s: DCI %i of %d total DCIs found --> rnti %x : format %d\n",
      ue->Mod_id,frame_rx%1024,nr_slot_rx,nr_mode_string[ue->UE_mode[gNB_id]],
      i + 1,
      dci_cnt,
      dci_ind->dci_list[i].rnti,
      dci_ind->dci_list[i].dci_format);
  }
  ue->pdcch_vars[proc->thread_id][gNB_id]->dci_received += dci_cnt;

  dci_ind->number_of_dcis = dci_cnt;

  // fill dl_indication message
  nr_fill_dl_indication(&dl_indication, dci_ind, NULL, proc, ue, gNB_id, phy_pdcch_config);
  //  send to mac
  ue->if_inst->dl_indication(&dl_indication, NULL);


  stop_meas(&ue->dlsch_rx_pdcch_stats);
    
  VCD_SIGNAL_DUMPER_DUMP_FUNCTION_BY_NAME(VCD_SIGNAL_DUMPER_FUNCTIONS_UE_PDCCH_PROCEDURES, VCD_FUNCTION_OUT);
  return(dci_cnt);
}
#endif // NR_PDCCH_SCHED

int nr_ue_pdsch_procedures(PHY_VARS_NR_UE *ue, UE_nr_rxtx_proc_t *proc, int gNB_id, PDSCH_t pdsch, NR_UE_DLSCH_t *dlsch0, NR_UE_DLSCH_t *dlsch1) {

  int frame_rx = proc->frame_rx;
  int nr_slot_rx = proc->nr_slot_rx;
  int m;
  int i_mod,gNB_id_i,dual_stream_UE;
  int first_symbol_flag=0;

  if (!dlsch0)
    return 0;
  if (dlsch0->active == 0)
    return 0;

  if (!dlsch1)  {
    int harq_pid = dlsch0->current_harq_pid;
    NR_DL_UE_HARQ_t *dlsch0_harq = dlsch0->harq_processes[harq_pid];
    uint16_t BWPStart       = dlsch0_harq->BWPStart;
    uint16_t pdsch_start_rb = dlsch0_harq->start_rb;
    uint16_t pdsch_nb_rb    = dlsch0_harq->nb_rb;
    uint16_t s0             = dlsch0_harq->start_symbol;
    uint16_t s1             = dlsch0_harq->nb_symbols;
    bool is_SI              = dlsch0->rnti_type == _SI_RNTI_;

    LOG_D(PHY,"[UE %d] PDSCH type %d active in nr_slot_rx %d, harq_pid %d (%d), rb_start %d, nb_rb %d, symbol_start %d, nb_symbols %d, DMRS mask %x, Nl %d\n",
          ue->Mod_id,pdsch,nr_slot_rx,harq_pid,dlsch0_harq->status,pdsch_start_rb,pdsch_nb_rb,s0,s1,dlsch0_harq->dlDmrsSymbPos, dlsch0_harq->Nl);

    for (m = s0; m < (s0 +s1); m++) {
      if (dlsch0_harq->dlDmrsSymbPos & (1 << m)) {
        for (uint8_t aatx=0; aatx<dlsch0_harq->Nl; aatx++) {//for MIMO Config: it shall loop over no_layers
          LOG_D(PHY,"PDSCH Channel estimation gNB id %d, PDSCH antenna port %d, slot %d, symbol %d\n",0,aatx,nr_slot_rx,m);
          nr_pdsch_channel_estimation(ue,
                                      proc,
                                      gNB_id,
                                      is_SI,
                                      nr_slot_rx,
                                      get_dmrs_port(aatx,dlsch0_harq->dmrs_ports),
                                      m,
                                      dlsch0_harq->nscid,
                                      dlsch0_harq->dlDmrsScramblingId,
                                      BWPStart,
                                      dlsch0_harq->dmrsConfigType,
                                      ue->frame_parms.first_carrier_offset+(BWPStart + pdsch_start_rb)*12,
                                      pdsch_nb_rb);
#if 0
          ///LOG_M: the channel estimation
          int nr_frame_rx = proc->frame_rx;
          char filename[100];
          for (uint8_t aarx=0; aarx<ue->frame_parms.nb_antennas_rx; aarx++) {
            sprintf(filename,"PDSCH_CHANNEL_frame%d_slot%d_sym%d_port%d_rx%d.m", nr_frame_rx, nr_slot_rx, m, aatx,aarx);
            int **dl_ch_estimates = ue->pdsch_vars[proc->thread_id][gNB_id]->dl_ch_estimates;
            LOG_M(filename,"channel_F",&dl_ch_estimates[aatx*ue->frame_parms.nb_antennas_rx+aarx][ue->frame_parms.ofdm_symbol_size*m],ue->frame_parms.ofdm_symbol_size, 1, 1);
          }
#endif
        }
      }
    }

    if (ue->chest_time == 1) { // averaging time domain channel estimates
      nr_chest_time_domain_avg(&ue->frame_parms,
                               ue->pdsch_vars[proc->thread_id][gNB_id]->dl_ch_estimates,
                               dlsch0_harq->nb_symbols,
                               dlsch0_harq->start_symbol,
                               dlsch0_harq->dlDmrsSymbPos,
                               pdsch_nb_rb);
    }

    uint16_t first_symbol_with_data = s0;
    uint32_t dmrs_data_re;

    if (dlsch0_harq->dmrsConfigType == NFAPI_NR_DMRS_TYPE1)
      dmrs_data_re = 12 - 6 * dlsch0_harq->n_dmrs_cdm_groups;
    else
      dmrs_data_re = 12 - 4 * dlsch0_harq->n_dmrs_cdm_groups;

    while ((dmrs_data_re == 0) && (dlsch0_harq->dlDmrsSymbPos & (1 << first_symbol_with_data))) {
      first_symbol_with_data++;
    }

    start_meas(&ue->rx_pdsch_stats);
    for (m = s0; m < (s1 + s0); m++) {
 
      dual_stream_UE = 0;
      gNB_id_i = gNB_id+1;
      i_mod = 0;
      if (m==first_symbol_with_data)
        first_symbol_flag = 1;
      else
        first_symbol_flag = 0;

      uint8_t slot = 0;
      if(m >= ue->frame_parms.symbols_per_slot>>1)
        slot = 1;
      start_meas(&ue->dlsch_llr_stats_parallelization[proc->thread_id][slot]);
      // process DLSCH received symbols in the slot
      // symbol by symbol processing (if data/DMRS are multiplexed is checked inside the function)
      if (pdsch == PDSCH || pdsch == SI_PDSCH || pdsch == RA_PDSCH) {
        if (nr_rx_pdsch(ue,
                        proc,
                        pdsch,
                        gNB_id,
                        gNB_id_i,
                        frame_rx,
                        nr_slot_rx,
                        m,
                        first_symbol_flag,
                        dual_stream_UE,
                        i_mod,
                        harq_pid) < 0)
          return -1;
      } else AssertFatal(1==0,"Not RA_PDSCH, SI_PDSCH or PDSCH\n");

      stop_meas(&ue->dlsch_llr_stats_parallelization[proc->thread_id][slot]);
      if (cpumeas(CPUMEAS_GETSTATE))
        LOG_D(PHY, "[AbsSFN %d.%d] LLR Computation Symbol %d %5.2f \n",frame_rx,nr_slot_rx,m,ue->dlsch_llr_stats_parallelization[proc->thread_id][slot].p_time/(cpuf*1000.0));
      if(first_symbol_flag) {
        proc->first_symbol_available = 1;
      }
    } // CRNTI active
    stop_meas(&ue->rx_pdsch_stats);
  }
  return 0;
}

bool nr_ue_dlsch_procedures(PHY_VARS_NR_UE *ue,
                            UE_nr_rxtx_proc_t *proc,
                            int gNB_id,
                            PDSCH_t pdsch,
                            NR_UE_DLSCH_t *dlsch0,
                            NR_UE_DLSCH_t *dlsch1,
                            int *dlsch_errors) {

  if (dlsch0==NULL)
    AssertFatal(0,"dlsch0 should be defined at this level \n");
  bool dec = false;
  int harq_pid = dlsch0->current_harq_pid;
  int frame_rx = proc->frame_rx;
  int nr_slot_rx = proc->nr_slot_rx;
  uint32_t ret = UINT32_MAX, ret1 = UINT32_MAX;
  NR_UE_PDSCH *pdsch_vars;
  uint16_t dmrs_len = get_num_dmrs(dlsch0->harq_processes[dlsch0->current_harq_pid]->dlDmrsSymbPos);
  nr_downlink_indication_t dl_indication;
  fapi_nr_rx_indication_t *rx_ind = calloc(1, sizeof(*rx_ind));
  uint16_t number_pdus = 1;
  // params for UL time alignment procedure
  NR_UL_TIME_ALIGNMENT_t *ul_time_alignment = &ue->ul_time_alignment[gNB_id];

  uint8_t is_cw0_active = dlsch0->harq_processes[harq_pid]->status;
  uint16_t nb_symb_sch = dlsch0->harq_processes[harq_pid]->nb_symbols;
  uint16_t start_symbol = dlsch0->harq_processes[harq_pid]->start_symbol;
  uint8_t dmrs_type = dlsch0->harq_processes[harq_pid]->dmrsConfigType;

  uint8_t nb_re_dmrs;
  if (dmrs_type==NFAPI_NR_DMRS_TYPE1) {
    nb_re_dmrs = 6*dlsch0->harq_processes[harq_pid]->n_dmrs_cdm_groups;
  }
  else {
    nb_re_dmrs = 4*dlsch0->harq_processes[harq_pid]->n_dmrs_cdm_groups;
  }

  uint8_t is_cw1_active = 0;
  if(dlsch1)
    is_cw1_active = dlsch1->harq_processes[harq_pid]->status;

  LOG_D(PHY,"AbsSubframe %d.%d Start LDPC Decoder for CW0 [harq_pid %d] ? %d \n", frame_rx%1024, nr_slot_rx, harq_pid, is_cw0_active);
  LOG_D(PHY,"AbsSubframe %d.%d Start LDPC Decoder for CW1 [harq_pid %d] ? %d \n", frame_rx%1024, nr_slot_rx, harq_pid, is_cw1_active);

  if(is_cw0_active && is_cw1_active)
    {
      dlsch0->Kmimo = 2;
      dlsch1->Kmimo = 2;
    }
  else
    {
      dlsch0->Kmimo = 1;
    }
  if (1) {
    switch (pdsch) {
    case SI_PDSCH:
    case RA_PDSCH:
    case P_PDSCH:
    case PDSCH:
      pdsch_vars = ue->pdsch_vars[proc->thread_id][gNB_id];
      break;
    case PMCH:
    case PDSCH1:
      LOG_E(PHY,"Illegal PDSCH %d for ue_pdsch_procedures\n",pdsch);
      pdsch_vars = NULL;
      return false;
      break;
    default:
      pdsch_vars = NULL;
      return false;
      break;

    }
    if (frame_rx < *dlsch_errors)
      *dlsch_errors=0;

    if (pdsch == RA_PDSCH) {
      if (ue->prach_resources[gNB_id]!=NULL)
        dlsch0->rnti = ue->prach_resources[gNB_id]->ra_RNTI;
      else {
        LOG_E(PHY,"[UE %d] Frame %d, nr_slot_rx %d: FATAL, prach_resources is NULL\n", ue->Mod_id, frame_rx, nr_slot_rx);
        //mac_xface->macphy_exit("prach_resources is NULL");
        return false;
      }
    }

    // exit dlsch procedures as there are no active dlsch
    if (is_cw0_active != ACTIVE && is_cw1_active != ACTIVE)
      return false;

    // start ldpc decode for CW 0
    dlsch0->harq_processes[harq_pid]->G = nr_get_G(dlsch0->harq_processes[harq_pid]->nb_rb,
                                                   nb_symb_sch,
                                                   nb_re_dmrs,
                                                   dmrs_len,
                                                   dlsch0->harq_processes[harq_pid]->Qm,
                                                   dlsch0->harq_processes[harq_pid]->Nl);

      start_meas(&ue->dlsch_unscrambling_stats);
      nr_dlsch_unscrambling(pdsch_vars->llr[0],
                            dlsch0->harq_processes[harq_pid]->G,
                            0,
                            ue->frame_parms.Nid_cell,
                            dlsch0->rnti);
      

      stop_meas(&ue->dlsch_unscrambling_stats);


#if 0
      LOG_I(PHY," ------ start ldpc decoder for AbsSubframe %d.%d / %d  ------  \n", frame_rx, nr_slot_rx, harq_pid);
      LOG_I(PHY,"start ldpc decode for CW 0 for AbsSubframe %d.%d / %d --> nb_rb %d \n", frame_rx, nr_slot_rx, harq_pid, dlsch0->harq_processes[harq_pid]->nb_rb);
      LOG_I(PHY,"start ldpc decode for CW 0 for AbsSubframe %d.%d / %d  --> rb_alloc_even %x \n", frame_rx, nr_slot_rx, harq_pid, dlsch0->harq_processes[harq_pid]->rb_alloc_even);
      LOG_I(PHY,"start ldpc decode for CW 0 for AbsSubframe %d.%d / %d  --> Qm %d \n", frame_rx, nr_slot_rx, harq_pid, dlsch0->harq_processes[harq_pid]->Qm);
      LOG_I(PHY,"start ldpc decode for CW 0 for AbsSubframe %d.%d / %d  --> Nl %d \n", frame_rx, nr_slot_rx, harq_pid, dlsch0->harq_processes[harq_pid]->Nl);
      LOG_I(PHY,"start ldpc decode for CW 0 for AbsSubframe %d.%d / %d  --> G  %d \n", frame_rx, nr_slot_rx, harq_pid, dlsch0->harq_processes[harq_pid]->G);
      LOG_I(PHY,"start ldpc decode for CW 0 for AbsSubframe %d.%d / %d  --> Kmimo  %d \n", frame_rx, nr_slot_rx, harq_pid, dlsch0->Kmimo);
      LOG_I(PHY,"start ldpc decode for CW 0 for AbsSubframe %d.%d / %d  --> Pdcch Sym  %d \n", frame_rx, nr_slot_rx, harq_pid, ue->pdcch_vars[proc->thread_id][gNB_id]->num_pdcch_symbols);
#endif


   start_meas(&ue->dlsch_decoding_stats[proc->thread_id]);

    ret = nr_dlsch_decoding(ue,
                            proc,
                            gNB_id,
                            pdsch_vars->llr[0],
                            &ue->frame_parms,
                            dlsch0,
                            dlsch0->harq_processes[harq_pid],
                            frame_rx,
                            nb_symb_sch,
                            nr_slot_rx,
                            harq_pid,
                            pdsch==PDSCH);

    LOG_T(PHY,"dlsch decoding, ret = %d\n", ret);


    if(ret<dlsch0->max_ldpc_iterations+1)
      dec = true;

    switch (pdsch) {
      case RA_PDSCH:
        nr_fill_dl_indication(&dl_indication, NULL, rx_ind, proc, ue, gNB_id, NULL);
        nr_fill_rx_indication(rx_ind, FAPI_NR_RX_PDU_TYPE_RAR, gNB_id, ue, dlsch0, NULL, number_pdus, proc, NULL);
        ue->UE_mode[gNB_id] = RA_RESPONSE;
        break;
      case PDSCH:
        nr_fill_dl_indication(&dl_indication, NULL, rx_ind, proc, ue, gNB_id, NULL);
        nr_fill_rx_indication(rx_ind, FAPI_NR_RX_PDU_TYPE_DLSCH, gNB_id, ue, dlsch0, NULL, number_pdus, proc, NULL);
        break;
      case SI_PDSCH:
        nr_fill_dl_indication(&dl_indication, NULL, rx_ind, proc, ue, gNB_id, NULL);
        nr_fill_rx_indication(rx_ind, FAPI_NR_RX_PDU_TYPE_SIB, gNB_id, ue, dlsch0, NULL, number_pdus, proc, NULL);
        break;
      default:
        break;
    }

    LOG_D(PHY, "In %s DL PDU length in bits: %d, in bytes: %d \n", __FUNCTION__, dlsch0->harq_processes[harq_pid]->TBS, dlsch0->harq_processes[harq_pid]->TBS / 8);

    stop_meas(&ue->dlsch_decoding_stats[proc->thread_id]);
    if (cpumeas(CPUMEAS_GETSTATE))  {
      LOG_D(PHY, " --> Unscrambling for CW0 %5.3f\n",
            (ue->dlsch_unscrambling_stats.p_time)/(cpuf*1000.0));
      LOG_D(PHY, "AbsSubframe %d.%d --> LDPC Decoding for CW0 %5.3f\n",
            frame_rx%1024, nr_slot_rx,(ue->dlsch_decoding_stats[proc->thread_id].p_time)/(cpuf*1000.0));
    }

    if(is_cw1_active) {
      // start ldpc decode for CW 1
      dlsch1->harq_processes[harq_pid]->G = nr_get_G(dlsch1->harq_processes[harq_pid]->nb_rb,
                                                     nb_symb_sch,
                                                     nb_re_dmrs,
                                                     dmrs_len,
                                                     dlsch1->harq_processes[harq_pid]->Qm,
                                                     dlsch1->harq_processes[harq_pid]->Nl);
      start_meas(&ue->dlsch_unscrambling_stats);
      nr_dlsch_unscrambling(pdsch_vars->llr[1],
                            dlsch1->harq_processes[harq_pid]->G,
                            0,
                            ue->frame_parms.Nid_cell,
                            dlsch1->rnti);
      stop_meas(&ue->dlsch_unscrambling_stats);

#if 0
          LOG_I(PHY,"start ldpc decode for CW 1 for AbsSubframe %d.%d / %d --> nb_rb %d \n", frame_rx, nr_slot_rx, harq_pid, dlsch1->harq_processes[harq_pid]->nb_rb);
          LOG_I(PHY,"start ldpc decode for CW 1 for AbsSubframe %d.%d / %d  --> rb_alloc_even %x \n", frame_rx, nr_slot_rx, harq_pid, dlsch1->harq_processes[harq_pid]->rb_alloc_even);
          LOG_I(PHY,"start ldpc decode for CW 1 for AbsSubframe %d.%d / %d  --> Qm %d \n", frame_rx, nr_slot_rx, harq_pid, dlsch1->harq_processes[harq_pid]->Qm);
          LOG_I(PHY,"start ldpc decode for CW 1 for AbsSubframe %d.%d / %d  --> Nl %d \n", frame_rx, nr_slot_rx, harq_pid, dlsch1->harq_processes[harq_pid]->Nl);
          LOG_I(PHY,"start ldpc decode for CW 1 for AbsSubframe %d.%d / %d  --> G  %d \n", frame_rx, nr_slot_rx, harq_pid, dlsch1->harq_processes[harq_pid]->G);
          LOG_I(PHY,"start ldpc decode for CW 1 for AbsSubframe %d.%d / %d  --> Kmimo  %d \n", frame_rx, nr_slot_rx, harq_pid, dlsch1->Kmimo);
          LOG_I(PHY,"start ldpc decode for CW 1 for AbsSubframe %d.%d / %d  --> Pdcch Sym  %d \n", frame_rx, nr_slot_rx, harq_pid, ue->pdcch_vars[proc->thread_id][gNB_id]->num_pdcch_symbols);
#endif

      start_meas(&ue->dlsch_decoding_stats[proc->thread_id]);


      ret1 = nr_dlsch_decoding(ue,
                               proc,
                               gNB_id,
                               pdsch_vars->llr[1],
                               &ue->frame_parms,
                               dlsch1,
                               dlsch1->harq_processes[harq_pid],
                               frame_rx,
                               nb_symb_sch,
                               nr_slot_rx,
                               harq_pid,
                               pdsch==PDSCH);//proc->decoder_switch
      LOG_T(PHY,"CW dlsch decoding, ret1 = %d\n", ret1);

      stop_meas(&ue->dlsch_decoding_stats[proc->thread_id]);
      if (cpumeas(CPUMEAS_GETSTATE)) {
        LOG_D(PHY, " --> Unscrambling for CW1 %5.3f\n",
              (ue->dlsch_unscrambling_stats.p_time)/(cpuf*1000.0));
        LOG_D(PHY, "AbsSubframe %d.%d --> ldpc Decoding for CW1 %5.3f\n",
              frame_rx%1024, nr_slot_rx,(ue->dlsch_decoding_stats[proc->thread_id].p_time)/(cpuf*1000.0));
        }
    LOG_D(PHY, "harq_pid: %d, TBS expected dlsch1: %d \n", harq_pid, dlsch1->harq_processes[harq_pid]->TBS);
    }
    //  send to mac
    if (ue->if_inst && ue->if_inst->dl_indication) {
      ue->if_inst->dl_indication(&dl_indication, ul_time_alignment);
    }

    if (ue->mac_enabled == 1) { // TODO: move this from PHY to MAC layer!

      /* Time Alignment procedure
      // - UE processing capability 1
      // - Setting the TA update to be applied after the reception of the TA command
      // - Timing adjustment computed according to TS 38.213 section 4.2
      // - Durations of N1 and N2 symbols corresponding to PDSCH and PUSCH are
      //   computed according to sections 5.3 and 6.4 of TS 38.214 */
      const int numerology = ue->frame_parms.numerology_index;
      const int ofdm_symbol_size = ue->frame_parms.ofdm_symbol_size;
      const int nb_prefix_samples = ue->frame_parms.nb_prefix_samples;
      const int samples_per_subframe = ue->frame_parms.samples_per_subframe;
      const int slots_per_frame = ue->frame_parms.slots_per_frame;
      const int slots_per_subframe = ue->frame_parms.slots_per_subframe;

      const double tc_factor = 1.0 / samples_per_subframe;
      const uint16_t bw_scaling = get_bw_scaling(ofdm_symbol_size);

      const int Ta_max = 3846; // Max value of 12 bits TA Command
      const double N_TA_max = Ta_max * bw_scaling * tc_factor;

      NR_UE_MAC_INST_t *mac = get_mac_inst(0);
      NR_BWP_Id_t dl_bwp = mac->DL_BWP_Id;
      NR_BWP_Id_t ul_bwp = mac->UL_BWP_Id;

      NR_PUSCH_TimeDomainResourceAllocationList_t *pusch_TimeDomainAllocationList = NULL;
      if(ul_bwp){
        if (mac->ULbwp[ul_bwp-1] &&
            mac->ULbwp[ul_bwp-1]->bwp_Dedicated &&
            mac->ULbwp[ul_bwp-1]->bwp_Dedicated->pusch_Config &&
            mac->ULbwp[ul_bwp-1]->bwp_Dedicated->pusch_Config->choice.setup &&
            mac->ULbwp[ul_bwp-1]->bwp_Dedicated->pusch_Config->choice.setup->pusch_TimeDomainAllocationList) {
          pusch_TimeDomainAllocationList = mac->ULbwp[ul_bwp-1]->bwp_Dedicated->pusch_Config->choice.setup->pusch_TimeDomainAllocationList->choice.setup;
        }
        else if (mac->ULbwp[ul_bwp-1] &&
                 mac->ULbwp[ul_bwp-1]->bwp_Common &&
                 mac->ULbwp[ul_bwp-1]->bwp_Common->pusch_ConfigCommon &&
                 mac->ULbwp[ul_bwp-1]->bwp_Common->pusch_ConfigCommon->choice.setup &&
                 mac->ULbwp[ul_bwp-1]->bwp_Common->pusch_ConfigCommon->choice.setup->pusch_TimeDomainAllocationList) {
          pusch_TimeDomainAllocationList = mac->ULbwp[ul_bwp-1]->bwp_Common->pusch_ConfigCommon->choice.setup->pusch_TimeDomainAllocationList;
        }
      }
      else if (mac->scc_SIB &&
               mac->scc_SIB->uplinkConfigCommon &&
               mac->scc_SIB->uplinkConfigCommon->initialUplinkBWP.pusch_ConfigCommon &&
               mac->scc_SIB->uplinkConfigCommon->initialUplinkBWP.pusch_ConfigCommon->choice.setup &&
               mac->scc_SIB->uplinkConfigCommon->initialUplinkBWP.pusch_ConfigCommon->choice.setup->pusch_TimeDomainAllocationList) {
        pusch_TimeDomainAllocationList = mac->scc_SIB->uplinkConfigCommon->initialUplinkBWP.pusch_ConfigCommon->choice.setup->pusch_TimeDomainAllocationList;
      }
      long mapping_type_ul = pusch_TimeDomainAllocationList ? pusch_TimeDomainAllocationList->list.array[0]->mappingType : NR_PUSCH_TimeDomainResourceAllocation__mappingType_typeA;

      NR_PDSCH_Config_t *pdsch_Config = NULL;
      NR_PDSCH_TimeDomainResourceAllocationList_t *pdsch_TimeDomainAllocationList = NULL;
      if(dl_bwp){
        pdsch_Config = (mac->DLbwp[dl_bwp-1] && mac->DLbwp[dl_bwp-1]->bwp_Dedicated->pdsch_Config->choice.setup) ? mac->DLbwp[dl_bwp-1]->bwp_Dedicated->pdsch_Config->choice.setup : NULL;
        if (mac->DLbwp[dl_bwp-1] && mac->DLbwp[dl_bwp-1]->bwp_Dedicated->pdsch_Config->choice.setup->pdsch_TimeDomainAllocationList)
          pdsch_TimeDomainAllocationList = pdsch_Config->pdsch_TimeDomainAllocationList->choice.setup;
        else if (mac->DLbwp[dl_bwp-1] && mac->DLbwp[dl_bwp-1]->bwp_Common->pdsch_ConfigCommon->choice.setup->pdsch_TimeDomainAllocationList)
          pdsch_TimeDomainAllocationList = mac->DLbwp[dl_bwp-1]->bwp_Common->pdsch_ConfigCommon->choice.setup->pdsch_TimeDomainAllocationList;
      }
      else if (mac->scc_SIB && mac->scc_SIB->downlinkConfigCommon.initialDownlinkBWP.pdsch_ConfigCommon->choice.setup)
        pdsch_TimeDomainAllocationList = mac->scc_SIB->downlinkConfigCommon.initialDownlinkBWP.pdsch_ConfigCommon->choice.setup->pdsch_TimeDomainAllocationList;
      long mapping_type_dl = pdsch_TimeDomainAllocationList ? pdsch_TimeDomainAllocationList->list.array[0]->mappingType : NR_PDSCH_TimeDomainResourceAllocation__mappingType_typeA;

      NR_DMRS_DownlinkConfig_t *NR_DMRS_dlconfig = NULL;
      if (pdsch_Config) {
        if (mapping_type_dl == NR_PDSCH_TimeDomainResourceAllocation__mappingType_typeA)
          NR_DMRS_dlconfig = (NR_DMRS_DownlinkConfig_t *)pdsch_Config->dmrs_DownlinkForPDSCH_MappingTypeA->choice.setup;
        else
          NR_DMRS_dlconfig = (NR_DMRS_DownlinkConfig_t *)pdsch_Config->dmrs_DownlinkForPDSCH_MappingTypeB->choice.setup;
      }

      pdsch_dmrs_AdditionalPosition_t add_pos_dl = pdsch_dmrs_pos2;
      if (NR_DMRS_dlconfig && NR_DMRS_dlconfig->dmrs_AdditionalPosition)
        add_pos_dl = *NR_DMRS_dlconfig->dmrs_AdditionalPosition;

      /* PDSCH decoding time N_1 for processing capability 1 */
      int N_1;

      if (add_pos_dl == pdsch_dmrs_pos0)
        N_1 = pdsch_N_1_capability_1[numerology][1];
      else if (add_pos_dl == pdsch_dmrs_pos1 || add_pos_dl == pdsch_dmrs_pos2)
        N_1 = pdsch_N_1_capability_1[numerology][2];
      else
        N_1 = pdsch_N_1_capability_1[numerology][3];

      /* PUSCH preapration time N_2 for processing capability 1 */
      const int N_2 = pusch_N_2_timing_capability_1[numerology][1];

      /* d_1_1 depending on the number of PDSCH symbols allocated */
      const int d = 0; // TODO number of overlapping symbols of the scheduling PDCCH and the scheduled PDSCH
      int d_1_1 = 0;
      if (mapping_type_dl == NR_PDSCH_TimeDomainResourceAllocation__mappingType_typeA)
       if (nb_symb_sch + start_symbol < 7)
          d_1_1 = 7 - (nb_symb_sch + start_symbol);
        else
          d_1_1 = 0;
      else // mapping type B
        switch (nb_symb_sch){
          case 7: d_1_1 = 0; break;
          case 4: d_1_1 = 3; break;
          case 2: d_1_1 = 3 + d; break;
          default: break;
        }

      /* d_2_1 */
      int d_2_1;
      if (mapping_type_ul == NR_PUSCH_TimeDomainResourceAllocation__mappingType_typeB && start_symbol != 0)
        d_2_1 = 0;
      else
        d_2_1 = 1;

      /* d_2_2 */
      const double d_2_2 = pusch_d_2_2_timing_capability_1[numerology][1];

      /* N_t_1 time duration in msec of N_1 symbols corresponding to a PDSCH reception time
      // N_t_2 time duration in msec of N_2 symbols corresponding to a PUSCH preparation time */
      double N_t_1 = (N_1 + d_1_1) * (ofdm_symbol_size + nb_prefix_samples) * tc_factor;
      double N_t_2 = (N_2 + d_2_1) * (ofdm_symbol_size + nb_prefix_samples) * tc_factor;
      if (N_t_2 < d_2_2) N_t_2 = d_2_2;

      /* Time alignment procedure */
      // N_t_1 + N_t_2 + N_TA_max must be in msec
      const double t_subframe = 1.0; // subframe duration of 1 msec
      const int ul_tx_timing_adjustment = 1 + (int)ceil(slots_per_subframe*(N_t_1 + N_t_2 + N_TA_max + 0.5)/t_subframe);

      if (ul_time_alignment->apply_ta == 1){
        ul_time_alignment->ta_slot = (nr_slot_rx + ul_tx_timing_adjustment) % slots_per_frame;
        if (nr_slot_rx + ul_tx_timing_adjustment > slots_per_frame){
          ul_time_alignment->ta_frame = (frame_rx + 1) % 1024;
        } else {
          ul_time_alignment->ta_frame = frame_rx;
        }
        // reset TA flag
        ul_time_alignment->apply_ta = 0;
        LOG_D(PHY,"Frame %d slot %d -- Starting UL time alignment procedures. TA update will be applied at frame %d slot %d\n",
             frame_rx, nr_slot_rx, ul_time_alignment->ta_frame, ul_time_alignment->ta_slot);
      }
    }
  }
  return dec;
}


/*!
 * \brief This is the UE synchronize thread.
 * It performs band scanning and synchonization.
 * \param arg is a pointer to a \ref PHY_VARS_NR_UE structure.
 * \returns a pointer to an int. The storage is not on the heap and must not be freed.
 */
#ifdef UE_SLOT_PARALLELISATION
#define FIFO_PRIORITY   40
void *UE_thread_slot1_dl_processing(void *arg) {

  static __thread int UE_dl_slot1_processing_retval;
  struct rx_tx_thread_data *rtd = arg;
  UE_nr_rxtx_proc_t *proc = rtd->proc;
  PHY_VARS_NR_UE    *ue   = rtd->UE;

  uint8_t pilot1;

  proc->instance_cnt_slot1_dl_processing=-1;
  proc->nr_slot_rx = proc->sub_frame_start * ue->frame_parms.slots_per_subframe;

  char threadname[256];
  sprintf(threadname,"UE_thread_slot1_dl_processing_%d", proc->sub_frame_start);

  cpu_set_t cpuset;
  CPU_ZERO(&cpuset);
  if ( (proc->sub_frame_start+1)%RX_NB_TH == 0 && threads.slot1_proc_one != -1 )
    CPU_SET(threads.slot1_proc_one, &cpuset);
  if ( RX_NB_TH > 1 && (proc->sub_frame_start+1)%RX_NB_TH == 1 && threads.slot1_proc_two != -1 )
    CPU_SET(threads.slot1_proc_two, &cpuset);
  if ( RX_NB_TH > 2 && (proc->sub_frame_start+1)%RX_NB_TH == 2 && threads.slot1_proc_three != -1 )
    CPU_SET(threads.slot1_proc_three, &cpuset);

  init_thread(900000,1000000 , FIFO_PRIORITY-1, &cpuset,
	      threadname);

  while (!oai_exit) {
    if (pthread_mutex_lock(&proc->mutex_slot1_dl_processing) != 0) {
      LOG_E( PHY, "[SCHED][UE] error locking mutex for UE slot1 dl processing\n" );
      exit_fun("nothing to add");
    }
    while (proc->instance_cnt_slot1_dl_processing < 0) {
      // most of the time, the thread is waiting here
      pthread_cond_wait( &proc->cond_slot1_dl_processing, &proc->mutex_slot1_dl_processing );
    }
    if (pthread_mutex_unlock(&proc->mutex_slot1_dl_processing) != 0) {
      LOG_E( PHY, "[SCHED][UE] error unlocking mutex for UE slot1 dl processing \n" );
      exit_fun("nothing to add");
    }

    int frame_rx            = proc->frame_rx;
    uint8_t subframe_rx         = proc->nr_slot_rx / ue->frame_parms.slots_per_subframe;
    uint8_t next_subframe_rx    = (1 + subframe_rx) % NR_NUMBER_OF_SUBFRAMES_PER_FRAME;
    uint8_t next_subframe_slot0 = next_subframe_rx * ue->frame_parms.slots_per_subframe;

    uint8_t slot1  = proc->nr_slot_rx + 1;
    uint8_t pilot0 = 0;

    //printf("AbsSubframe %d.%d execute dl slot1 processing \n", frame_rx, nr_slot_rx);

    if (ue->frame_parms.Ncp == 0) {  // normal prefix
      pilot1 = 4;
    } else { // extended prefix
      pilot1 = 3;
    }

    /**** Slot1 FE Processing ****/

    start_meas(&ue->ue_front_end_per_slot_stat[proc->thread_id][1]);

    // I- start dl slot1 processing
    // do first symbol of next downlink nr_slot_rx for channel estimation
    /*
    // 1- perform FFT for pilot ofdm symbols first (ofdmSym0 next nr_slot_rx ofdmSym11)
    if (nr_subframe_select(&ue->frame_parms,next_nr_slot_rx) != SF_UL)
    {
    front_end_fft(ue,
    pilot0,
    next_subframe_slot0,
    0,
    0);
    }

    front_end_fft(ue,
    pilot1,
    slot1,
    0,
    0);
    */
    // 1- perform FFT
    for (int l=1; l<ue->frame_parms.symbols_per_slot>>1; l++)
      {
	//if( (l != pilot0) && (l != pilot1))
	{

	  start_meas(&ue->ofdm_demod_stats);
	  VCD_SIGNAL_DUMPER_DUMP_FUNCTION_BY_NAME(VCD_SIGNAL_DUMPER_FUNCTIONS_UE_SLOT_FEP, VCD_FUNCTION_IN);
	  //printf("AbsSubframe %d.%d FFT slot %d, symbol %d\n", frame_rx,nr_slot_rx,slot1,l);
	  front_end_fft(ue,
                        l,
                        slot1,
                        0,
                        0);
	  VCD_SIGNAL_DUMPER_DUMP_FUNCTION_BY_NAME(VCD_SIGNAL_DUMPER_FUNCTIONS_UE_SLOT_FEP, VCD_FUNCTION_OUT);
	  stop_meas(&ue->ofdm_demod_stats);
	}
      } // for l=1..l2

    if (nr_subframe_select(&ue->frame_parms,next_nr_slot_rx) != SF_UL)
      {
	//printf("AbsSubframe %d.%d FFT slot %d, symbol %d\n", frame_rx,nr_slot_rx,next_subframe_slot0,pilot0);
	front_end_fft(ue,
		      pilot0,
		      next_subframe_slot0,
		      0,
		      0);
      }

    // 2- perform Channel Estimation for slot1
    for (int l=1; l<ue->frame_parms.symbols_per_slot>>1; l++)
      {
	if(l == pilot1)
	  {
	    //wait until channel estimation for pilot0/slot1 is available
	    uint32_t wait = 0;
	    while(proc->chan_est_pilot0_slot1_available == 0)
	      {
		usleep(1);
		wait++;
	      }
	    //printf("[slot1 dl processing] ChanEst symbol %d slot %d wait%d\n",l,slot1,wait);
	  }
	//printf("AbsSubframe %d.%d ChanEst slot %d, symbol %d\n", frame_rx,nr_slot_rx,slot1,l);
	front_end_chanEst(ue,
			  l,
			  slot1,
			  0);
	ue_measurement_procedures(l-1,ue,proc,0,slot1,0,ue->mode);
      }
    //printf("AbsSubframe %d.%d ChanEst slot %d, symbol %d\n", frame_rx,nr_slot_rx,next_subframe_slot0,pilot0);
    front_end_chanEst(ue,
		      pilot0,
		      next_subframe_slot0,
		      0);

    if ( (nr_slot_rx == 0) && (ue->decode_MIB == 1))
      {
	ue_pbch_procedures(0,ue,proc,0);
      }

    proc->chan_est_slot1_available = 1;
    //printf("Set available slot 1channelEst to 1 AbsSubframe %d.%d \n",frame_rx,nr_slot_rx);
    //printf(" [slot1 dl processing] ==> FFT/CHanEst Done for AbsSubframe %d.%d \n", proc->frame_rx, proc->nr_slot_rx);

    //printf(" [slot1 dl processing] ==> Start LLR Comuptation slot1 for AbsSubframe %d.%d \n", proc->frame_rx, proc->nr_slot_rx);



    stop_meas(&ue->ue_front_end_per_slot_stat[proc->thread_id][1]);
    if (cpumeas(CPUMEAS_GETSTATE))
      LOG_D(PHY, "[AbsSFN %d.%d] Slot1: FFT + Channel Estimate + Pdsch Proc Slot0 %5.2f \n",frame_rx,nr_slot_rx,ue->ue_front_end_per_slot_stat[proc->thread_id][1].p_time/(cpuf*1000.0));

    //wait until pdcch is decoded
    uint32_t wait = 0;
    while(proc->dci_slot0_available == 0)
      {
        usleep(1);
        wait++;
      }
    //printf("[slot1 dl processing] AbsSubframe %d.%d LLR Computation Start wait DCI %d\n",frame_rx,nr_slot_rx,wait);

    /**** Pdsch Procedure Slot1 ****/
    // start slot1 thread for Pdsch Procedure (slot1)
    // do procedures for C-RNTI
    //printf("AbsSubframe %d.%d Pdsch Procedure (slot1)\n",frame_rx,nr_slot_rx);


    start_meas(&ue->pdsch_procedures_per_slot_stat[proc->thread_id][1]);

    // start slave thread for Pdsch Procedure (slot1)
    // do procedures for C-RNTI
    uint8_t gNB_id = 0;

    if (ue->dlsch[proc->thread_id][gNB_id][0]->active == 1) {
      //wait until first ofdm symbol is processed
      //wait = 0;
      //while(proc->first_symbol_available == 0)
      //{
      //    usleep(1);
      //    wait++;
      //}
      //printf("[slot1 dl processing] AbsSubframe %d.%d LLR Computation Start wait First Ofdm Sym %d\n",frame_rx,nr_slot_rx,wait);

      //VCD_SIGNAL_DUMPER_DUMP_FUNCTION_BY_NAME(VCD_SIGNAL_DUMPER_FUNCTIONS_PDSCH_PROC, VCD_FUNCTION_IN);
      ue_pdsch_procedures(ue,
			  proc,
			  gNB_id,
			  PDSCH,
			  ue->dlsch[proc->thread_id][gNB_id][0],
			  NULL,
			  (ue->frame_parms.symbols_per_slot>>1),
			  ue->frame_parms.symbols_per_slot-1,
			  abstraction_flag);
      LOG_D(PHY," ------ end PDSCH ChannelComp/LLR slot 0: AbsSubframe %d.%d ------  \n", frame_rx%1024, nr_slot_rx);
      LOG_D(PHY," ------ --> PDSCH Turbo Decoder slot 0/1: AbsSubframe %d.%d ------  \n", frame_rx%1024, nr_slot_rx);
    }

    // do procedures for SI-RNTI
    if ((ue->dlsch_SI[gNB_id]) && (ue->dlsch_SI[gNB_id]->active == 1)) {
      ue_pdsch_procedures(ue,
			  proc,
			  gNB_id,
			  SI_PDSCH,
			  ue->dlsch_SI[gNB_id],
			  NULL,
			  (ue->frame_parms.symbols_per_slot>>1),
			  ue->frame_parms.symbols_per_slot-1,
			  abstraction_flag);
    }

    // do procedures for P-RNTI
    if ((ue->dlsch_p[gNB_id]) && (ue->dlsch_p[gNB_id]->active == 1)) {
      ue_pdsch_procedures(ue,
			  proc,
			  gNB_id,
			  P_PDSCH,
			  ue->dlsch_p[gNB_id],
			  NULL,
			  (ue->frame_parms.symbols_per_slot>>1),
			  ue->frame_parms.symbols_per_slot-1,
			  abstraction_flag);
    }
    // do procedures for RA-RNTI
    if ((ue->dlsch_ra[gNB_id]) && (ue->dlsch_ra[gNB_id]->active == 1) && (UE_mode != PUSCH)) {
      ue_pdsch_procedures(ue,
			  proc,
			  gNB_id,
			  RA_PDSCH,
			  ue->dlsch_ra[gNB_id],
			  NULL,
			  (ue->frame_parms.symbols_per_slot>>1),
			  ue->frame_parms.symbols_per_slot-1,
			  abstraction_flag);
    }

    proc->llr_slot1_available=1;
    //printf("Set available LLR slot1 to 1 AbsSubframe %d.%d \n",frame_rx,nr_slot_rx);

    stop_meas(&ue->pdsch_procedures_per_slot_stat[proc->thread_id][1]);
    if (cpumeas(CPUMEAS_GETSTATE))
      LOG_D(PHY, "[AbsSFN %d.%d] Slot1: LLR Computation %5.2f \n",frame_rx,nr_slot_rx,ue->pdsch_procedures_per_slot_stat[proc->thread_id][1].p_time/(cpuf*1000.0));

    if (pthread_mutex_lock(&proc->mutex_slot1_dl_processing) != 0) {
      LOG_E( PHY, "[SCHED][UE] error locking mutex for UE RXTX\n" );
      exit_fun("noting to add");
    }
    proc->instance_cnt_slot1_dl_processing--;
    if (pthread_mutex_unlock(&proc->mutex_slot1_dl_processing) != 0) {
      LOG_E( PHY, "[SCHED][UE] error unlocking mutex for UE FEP Slo1\n" );
      exit_fun("noting to add");
    }
  }
  // thread finished
  free(arg);
  return &UE_dl_slot1_processing_retval;
}
#endif
int phy_procedures_nrUE_SL_RX(PHY_VARS_NR_UE *ue,
                           UE_nr_rxtx_proc_t *proc,
                           uint8_t synchRefUE_id,
                           notifiedFIFO_t *txFifo) {

  if (ue->is_synchronized_sl == 0)
    return (0);

  // TODO: Need to add rx SSB slot 2 (to Relay UE) and rx SSB slot 4 (Nearby UE) for resync
  uint32_t sl_bitmap_rx = ue->sync_ref ? 0x00001 : 0x00002; // Relay UE B rx slot 0, Nearby UE rx slot 1.
  if (((sl_bitmap_rx >> proc->nr_slot_rx) & 1) == 0) 
    return (0);

  int frame_rx = proc->frame_rx;
  int slot_rx = proc->nr_slot_rx;
  LOG_I(PHY,"SLOT_RX: %d \n", slot_rx);
  nr_ue_rrc_measurements(ue, proc, slot_rx);

  if ( getenv("RFSIMULATOR") != NULL ) {
    if (get_softmodem_params()->sl_mode == 2) {
      int frame_length_complex_samples = ue->frame_parms.samples_per_subframe * NR_NUMBER_OF_SUBFRAMES_PER_FRAME;
      for (int i = 0; i < frame_length_complex_samples; i++) {
        double sigma2_dB = 20 * log10((double)AMP / 4) - ue->snr;
        double sigma2 = pow(10, sigma2_dB / 10);
        for (int aa = 0; aa < ue->frame_parms.nb_antennas_rx; aa++) {
          ((short*) ue->common_vars.rxdata[aa])[2 * i] += (sqrt(sigma2 / 2) * gaussdouble(0.0, 1.0));
          ((short*) ue->common_vars.rxdata[aa])[2 * i + 1] += (sqrt(sigma2 / 2) * gaussdouble(0.0, 1.0));
        }
      }
    }
  }

  NR_UE_DLSCH_t   *slsch = ue->slsch_rx[proc->thread_id][synchRefUE_id][0];
  NR_DL_UE_HARQ_t *harq = NULL;
  int32_t **rxdataF = ue->common_vars.common_vars_rx_data_per_thread[0].rxdataF;
  uint64_t rx_offset = (slot_rx&3)*(ue->frame_parms.symbols_per_slot * ue->frame_parms.ofdm_symbol_size);

  static bool detect_new_dest;
  uint16_t node_id = get_softmodem_params()->node_number;
  uint32_t B_mul = get_B_multiplexed_value(&ue->frame_parms, slsch->harq_processes[0]);
  uint16_t Nidx = 1;
  for (unsigned char harq_pid = 0; harq_pid < 1; harq_pid++) {
    nr_ue_set_slsch_rx(ue, harq_pid);
    if (slsch->harq_processes[harq_pid]->status == ACTIVE) {
      harq = slsch->harq_processes[harq_pid];
      for (int aa = 0; aa < ue->frame_parms.nb_antennas_rx; aa++) {
        for (int ofdm_symbol = 0; ofdm_symbol < NR_NUMBER_OF_SYMBOLS_PER_SLOT; ofdm_symbol++) {
          nr_slot_fep_ul(&ue->frame_parms, ue->common_vars.rxdata[aa], &rxdataF[aa][rx_offset], ofdm_symbol, slot_rx, 0);
        }
        apply_nr_rotation_ul(&ue->frame_parms, rxdataF[aa], slot_rx, 0, NR_NUMBER_OF_SYMBOLS_PER_SLOT, link_type_sl);
      }
      uint32_t ret = nr_ue_slsch_rx_procedures(ue, harq_pid, frame_rx, slot_rx, rxdataF, B_mul, Nidx, proc);

      bool payload_type_string = true;
      bool polar_decoded = (ret < LDPC_MAX_LIMIT) ? true : false;
      uint16_t dest = (*harq->b_sci2 >> 32) & 0xFFFF;
      bool dest_matched = (dest == node_id);
      if (polar_decoded)
        LOG_D(PHY, "dest %u vs %u node_id for hex %lx\n", dest, node_id, *harq->b_sci2);
      if ((ret != -1) && dest_matched) {
        if (payload_type_string)
          validate_rx_payload_str(harq, slot_rx, polar_decoded);
        else
          validate_rx_payload(harq, frame_rx, slot_rx, polar_decoded);
      }
      if ((dest_matched == false) && (ue->sync_ref == 0) && (detect_new_dest == false)) {
        if(ue->slss->sl_timeoffsetssb_r16 == 2) {
          detect_new_dest = true;
          ue->sync_ref = 1;
          ue->slss->sl_timeoffsetssb_r16 = (node_id - 1) * 2 + 2;
          ue->slss->sl_timeoffsetssb_r16_copy = ue->slss->sl_timeoffsetssb_r16;
          init_mutex_of_relay_data();
        }
      }
      if ((dest_matched == false) && ue->sync_ref) {
        put_relay_data_to_buffer(harq->b, harq->b_sci2, harq->TBS);
      }
    }
  }
  LOG_D(PHY,"****** end Sidelink TX-Chain for AbsSlot %d.%d ******\n", frame_rx, slot_rx);
  return (0);
}

#define DEBUG_NR_PSSCHSIM
void /* The above code is likely defining a function called "validate_rx_payload" in the C programming
language. */
/* The above code is likely a function or method called "validate_rx_payload" written in the C
programming language. However, without the actual code implementation, it is not possible to
determine what the function does. */
validate_rx_payload(NR_DL_UE_HARQ_t *harq, int frame_rx, int slot_rx, bool polar_decoded) {
  unsigned int errors_bit = 0;
  unsigned int n_false_positive = 0;
#ifdef DEBUG_NR_PSSCHSIM
  unsigned char estimated_output_bit[HNA_SIZE];
  unsigned char test_input_bit[HNA_SIZE];
  unsigned char test_input[harq->TBS];
  uint32_t frame_tx = 0;
  uint32_t slot_tx = 0;
  unsigned char  randm_tx =0;
  static uint16_t sum_passed = 0;

  static uint16_t sum_failed = 0;
  uint8_t comparison_beg_byte = 4;
  uint8_t comparison_end_byte = 10;

  if (polar_decoded == true) {
    for (int i = 0; i < harq->TBS; i++)
      test_input[i] = (unsigned char) (i + 3);
    for (int i = 0; i < harq->TBS * 8; i++) {
      estimated_output_bit[i] = (harq->b[i / 8] & (1 << (i & 7))) >> (i & 7);
      test_input_bit[i] = (test_input[i / 8] & (1 << (i & 7))) >> (i & 7); // Further correct for multiple segments
      if (i % 8 == 0) {
        if (i == 8 * 0) slot_tx = harq->b[0];
        if (i == 8 * 1) frame_tx = harq->b[1];
        if (i == 8 * 2) frame_tx += (harq->b[2] << 8);
        if (i == 8 * 3) randm_tx = harq->b[3];
        if (i >= 8 * comparison_beg_byte)
          LOG_I(NR_PHY, "TxByte : %4c  vs  %4c : RxByte\n", test_input[i / 8], harq->b[i / 8]);
      }

      LOG_D(NR_PHY, "tx bit: %u, rx bit: %u\n", test_input_bit[i], estimated_output_bit[i]);
      bool limited_input = (i  >= 8 * comparison_beg_byte);
      if (i  >= 8 * comparison_end_byte)
        break;
      if ( limited_input && (estimated_output_bit[i] != test_input_bit[i])) {
        errors_bit++;
      }
    }

    LOG_I(NR_PHY, "TxRandm: %4u\n", randm_tx);
    LOG_I(NR_PHY, "TxFrame: %4u  vs  %4u : RxFrame\n", frame_tx, (uint32_t) frame_rx);
    LOG_I(NR_PHY, "TxSlot : %4u  vs  %4u : RxSlot \n", slot_tx,  (uint8_t) slot_rx);
  }
#endif
  if (errors_bit > 0 || polar_decoded == false) {
    n_false_positive++;
    ++sum_failed;
    LOG_I(NR_PHY, "errors_bit %u, polar_decoded %d\n", errors_bit, polar_decoded);
    LOG_I(NR_PHY, "PSSCH test NG with %d / %d = %4.2f\n", sum_passed, (sum_passed + sum_failed), (float) sum_passed / (float) (sum_passed + sum_failed));
  } else {
    ++sum_passed;
    LOG_I(NR_PHY, "PSSCH test OK with %d / %d = %4.2f\n", sum_passed, (sum_passed + sum_failed), (float) sum_passed / (float) (sum_passed + sum_failed));
    // if (slot_rx ==4)
    //   exit(0);
  }
}

void validate_rx_payload_str(NR_DL_UE_HARQ_t *harq, int slot, bool polar_decoded) {
  #define MAX_MSG_SIZE 1024
  #define AVG_MSG_SIZE 32
  unsigned int errors_bit = 0;
  char estimated_output_bit[HNA_SIZE];
  char test_input_bit[HNA_SIZE];
  unsigned int n_false_positive = 0;
  char default_input[MAX_MSG_SIZE] = {};
  char *sl_user_msg = get_softmodem_params()->sl_user_msg;
  char *test_input = (sl_user_msg != NULL) ? sl_user_msg : default_input;
  uint32_t default_msg_len = (sl_user_msg != NULL) ? 32 : MAX_MSG_SIZE;


 

  uint32_t test_msg_len = min(strlen((char *) harq->b), min(default_msg_len, harq->TBS));

  static uint16_t sum_passed = 0;
  static uint16_t sum_failed = 0;
  uint8_t comparison_end_byte = test_msg_len;

  static uint32_t total_bits = 0;
  if (polar_decoded == true) {
    for (int i = 0; i < test_msg_len * 8; i++) { //max tx string size is 32 bytes
				       //
						 //
      estimated_output_bit[i] = (harq->b[i / 8] & (1 << (i & 7))) >> (i & 7);
      test_input_bit[i] = (test_input[i / 8] & (1 << (i & 7))) >> (i & 7); // Further correct for multiple segments
      if(i % 8 == 0){
        LOG_D(NR_PHY, "TxByte : %c  vs  %c : RxByte\n", test_input[i / 8], harq->b[i / 8]);
      }
  #ifdef DEBUG_NR_PSSCHSIM
      LOG_D(NR_PHY, "tx bit: %u, rx bit: %u\n", test_input_bit[i], estimated_output_bit[i]);
  #endif
      if (i  >= 8 * comparison_end_byte)
        break;
      if (estimated_output_bit[i] != test_input_bit[i]) {
        if (sl_user_msg != NULL)
          errors_bit++;
      }
    }

    static char result[MAX_MSG_SIZE];
    for (int i = 0; i < test_msg_len; i++) {
      result[i] = harq->b[i];
      LOG_D(NR_PHY, "result[%d]=%c\n", i, result[i]);
    }
    char *usr_msg_ptr = &result[0];
    struct sockaddr_in remote_addr;
    memset(&remote_addr, 0, sizeof(remote_addr));
    remote_addr.sin_family = AF_INET;
    remote_addr.sin_port = htons(9999); 
    inet_pton(AF_INET, "127.0.0.1", &remote_addr.sin_addr);

    char *a = "a";

    ssize_t sent_len = sendto(udp_socket, usr_msg_ptr, test_msg_len, 0,
(struct sockaddr *)&remote_addr, sizeof(remote_addr));
    

    char msg_head_tail[128] = "Received your text! It says: ";

    memset(msg_head_tail, '+', min(128, strlen(msg_head_tail) + AVG_MSG_SIZE));
    LOG_I(NR_PHY, "%s\n", msg_head_tail);

    LOG_I(NR_PHY, "Received your text! It says: %s, %d\n", usr_msg_ptr,slot);
    LOG_I(NR_PHY, "Decoded_payload for slot %d: %s\n", slot, result);
    LOG_I(NR_PHY, "%s\n", msg_head_tail);
  }

  if (errors_bit > 0 || polar_decoded == false) {
    n_false_positive++;
    ++sum_failed;
    LOG_I(PHY,"errors_bit %u, polar_decoded %d\n", errors_bit, polar_decoded);
    LOG_I(PHY, "PSSCH test NG with %d / %d = %4.2f\n", sum_passed, (sum_passed + sum_failed), (float) sum_passed / (float) (sum_passed + sum_failed));
  } else if (sl_user_msg != NULL) {
    ++sum_passed;
    LOG_I(PHY, "PSSCH test OK with %d / %d = %4.2f\n", sum_passed, (sum_passed + sum_failed), (float) sum_passed / (float) (sum_passed + sum_failed));

  }
  total_bits = harq->G * (sum_passed + sum_failed);
  LOG_I(NR_PHY, "total bits: %d \n", total_bits);
}

int phy_procedures_nrUE_RX(PHY_VARS_NR_UE *ue,
                           UE_nr_rxtx_proc_t *proc,
                           uint8_t gNB_id,
                           NR_UE_PDCCH_CONFIG *phy_pdcch_config,
                           notifiedFIFO_t *txFifo) {

  int frame_rx = proc->frame_rx;
  int nr_slot_rx = proc->nr_slot_rx;
  fapi_nr_config_request_t *cfg = &ue->nrUE_config;

  NR_DL_FRAME_PARMS *fp = &ue->frame_parms;
  
  VCD_SIGNAL_DUMPER_DUMP_FUNCTION_BY_NAME(VCD_SIGNAL_DUMPER_FUNCTIONS_PHY_PROCEDURES_UE_RX, VCD_FUNCTION_IN);
  start_meas(&ue->phy_proc_rx[proc->thread_id]);

  LOG_D(PHY," ****** start RX-Chain for Frame.Slot %d.%d (energy %d dB)******  \n",
        frame_rx%1024, nr_slot_rx,
        dB_fixed(signal_energy(ue->common_vars.common_vars_rx_data_per_thread[proc->thread_id].rxdataF[0],2048*14)));

  // checking if current frame is compatible with SSB periodicity
  if (cfg->ssb_table.ssb_period == 0 ||
      !(frame_rx%(1<<(cfg->ssb_table.ssb_period-1)))){

    const int estimateSz = fp->symbols_per_slot * fp->ofdm_symbol_size;
    // loop over SSB blocks
    for(int ssb_index=0; ssb_index<fp->Lmax; ssb_index++) {
      uint32_t curr_mask = cfg->ssb_table.ssb_mask_list[ssb_index/32].ssb_mask;
      // check if if current SSB is transmitted
      if ((curr_mask >> (31-(ssb_index%32))) &0x01) {
        int ssb_start_symbol = nr_get_ssb_start_symbol(fp, ssb_index);
        int ssb_slot = ssb_start_symbol/fp->symbols_per_slot;
        int ssb_slot_2 = (cfg->ssb_table.ssb_period == 0) ? ssb_slot+(fp->slots_per_frame>>1) : -1;

        if (ssb_slot == nr_slot_rx ||
            ssb_slot_2 == nr_slot_rx) {

          VCD_SIGNAL_DUMPER_DUMP_FUNCTION_BY_NAME(VCD_SIGNAL_DUMPER_FUNCTIONS_UE_SLOT_FEP_PBCH, VCD_FUNCTION_IN);
          LOG_D(PHY," ------  PBCH ChannelComp/LLR: frame.slot %d.%d ------  \n", frame_rx%1024, nr_slot_rx);

          __attribute__ ((aligned(32))) struct complex16 dl_ch_estimates[fp->nb_antennas_rx][estimateSz];
          __attribute__ ((aligned(32))) struct complex16 dl_ch_estimates_time[fp->nb_antennas_rx][fp->ofdm_symbol_size];

          for (int i=1; i<4; i++) {
            nr_slot_fep(ue,
                        proc,
                        (ssb_start_symbol+i)%(fp->symbols_per_slot),
                        nr_slot_rx);

            start_meas(&ue->dlsch_channel_estimation_stats);
            nr_pbch_channel_estimation(ue, estimateSz, dl_ch_estimates,
                                       dl_ch_estimates_time, proc, gNB_id,
                                       nr_slot_rx, (ssb_start_symbol+i)%(fp->symbols_per_slot),
                                       i-1, ssb_index&7, ssb_slot_2 == nr_slot_rx);
            stop_meas(&ue->dlsch_channel_estimation_stats);
          }

          nr_ue_ssb_rsrp_measurements(ue, ssb_index, proc, nr_slot_rx);

          // resetting ssb index for PBCH detection if there is a stronger SSB index
          if(ue->measurements.ssb_rsrp_dBm[ssb_index] > ue->measurements.ssb_rsrp_dBm[fp->ssb_index])
            fp->ssb_index = ssb_index;

          if(ssb_index == fp->ssb_index) {

            LOG_D(PHY," ------  Decode MIB: frame.slot %d.%d ------  \n", frame_rx%1024, nr_slot_rx);
            nr_ue_pbch_procedures(gNB_id, ue, proc, estimateSz, dl_ch_estimates, phy_pdcch_config);

            if (ue->no_timing_correction==0) {
             LOG_D(PHY,"start adjust sync slot = %d no timing %d\n", nr_slot_rx, ue->no_timing_correction);
             nr_adjust_synch_ue(fp,
                                ue,
                                gNB_id,
                                fp->ofdm_symbol_size,
                                dl_ch_estimates_time,
                                frame_rx,
                                nr_slot_rx,
                                0,
                                16384);
            }
          }
          LOG_D(PHY, "Doing N0 measurements in %s\n", __FUNCTION__);
          nr_ue_rrc_measurements(ue, proc, nr_slot_rx);
          VCD_SIGNAL_DUMPER_DUMP_FUNCTION_BY_NAME(VCD_SIGNAL_DUMPER_FUNCTIONS_UE_SLOT_FEP_PBCH, VCD_FUNCTION_OUT);
        }
      }
    }
  }

  if ((frame_rx%64 == 0) && (nr_slot_rx==0)) {
    LOG_I(NR_PHY,"============================================\n");
    // fixed text + 8 HARQs rounds à 10 ("999999999/") + NULL
    // if we use 999999999 HARQs, that should be sufficient for at least 138 hours
    const size_t harq_output_len = 31 + 10 * 8 + 1;
    char output[harq_output_len];
    char *p = output;
    const char *end = output + harq_output_len;
    p += snprintf(p, end - p, "Harq round stats for Downlink: %d", ue->dl_stats[0]);
    for (int round = 1; round < 16 && (round < 3 || ue->dl_stats[round] != 0); ++round)
      p += snprintf(p, end - p,"/%d", ue->dl_stats[round]);
    LOG_I(NR_PHY,"%s/0\n", output);

    LOG_I(NR_PHY,"============================================\n");
  }

#ifdef NR_PDCCH_SCHED

  LOG_D(PHY," ------ --> PDCCH ChannelComp/LLR Frame.slot %d.%d ------  \n", frame_rx%1024, nr_slot_rx);
  VCD_SIGNAL_DUMPER_DUMP_FUNCTION_BY_NAME(VCD_SIGNAL_DUMPER_FUNCTIONS_UE_SLOT_FEP_PDCCH, VCD_FUNCTION_IN);

  uint8_t nb_symb_pdcch = phy_pdcch_config->nb_search_space > 0 ? phy_pdcch_config->pdcch_config[0].coreset.duration : 0;
  for (uint16_t l=0; l<nb_symb_pdcch; l++) {

    start_meas(&ue->ofdm_demod_stats);
    nr_slot_fep(ue,
                proc,
                l,
                nr_slot_rx);
  }

    // Hold the channel estimates in frequency domain.
  int32_t pdcch_est_size = ((((fp->symbols_per_slot*(fp->ofdm_symbol_size+LTE_CE_FILTER_LENGTH))+15)/16)*16);
  __attribute__ ((aligned(16))) int32_t pdcch_dl_ch_estimates[4*fp->nb_antennas_rx][pdcch_est_size];

  uint8_t dci_cnt = 0;
  for(int n_ss = 0; n_ss<phy_pdcch_config->nb_search_space; n_ss++) {
    for (uint16_t l=0; l<nb_symb_pdcch; l++) {

      // note: this only works if RBs for PDCCH are contigous!

      nr_pdcch_channel_estimation(ue,
                                  proc,
                                  gNB_id,
                                  nr_slot_rx,
                                  l,
                                  &phy_pdcch_config->pdcch_config[n_ss].coreset,
                                  fp->first_carrier_offset,
                                  phy_pdcch_config->pdcch_config[n_ss].BWPStart,
                                  pdcch_est_size,
                                  pdcch_dl_ch_estimates);

      stop_meas(&ue->ofdm_demod_stats);

    }
    dci_cnt = dci_cnt + nr_ue_pdcch_procedures(gNB_id, ue, proc, pdcch_est_size, pdcch_dl_ch_estimates, phy_pdcch_config, n_ss);
  }
  VCD_SIGNAL_DUMPER_DUMP_FUNCTION_BY_NAME(VCD_SIGNAL_DUMPER_FUNCTIONS_UE_SLOT_FEP_PDCCH, VCD_FUNCTION_OUT);

  if (dci_cnt > 0) {

    LOG_D(PHY,"[UE %d] Frame %d, nr_slot_rx %d: found %d DCIs\n", ue->Mod_id, frame_rx, nr_slot_rx, dci_cnt);

    NR_UE_DLSCH_t *dlsch = NULL;
    if (ue->dlsch[proc->thread_id][gNB_id][0]->active == 1){
      dlsch = ue->dlsch[proc->thread_id][gNB_id][0];
    } else if (ue->dlsch_SI[0]->active == 1){
      dlsch = ue->dlsch_SI[0];
    } else if (ue->dlsch_ra[0]->active == 1){
      dlsch = ue->dlsch_ra[0];
    }

    if (dlsch) {
      VCD_SIGNAL_DUMPER_DUMP_FUNCTION_BY_NAME(VCD_SIGNAL_DUMPER_FUNCTIONS_UE_SLOT_FEP_PDSCH, VCD_FUNCTION_IN);
      uint8_t harq_pid = dlsch->current_harq_pid;
      NR_DL_UE_HARQ_t *dlsch0_harq = dlsch->harq_processes[harq_pid];
      uint16_t nb_symb_sch = dlsch0_harq->nb_symbols;
      uint16_t start_symb_sch = dlsch0_harq->start_symbol;

      LOG_D(PHY," ------ --> PDSCH ChannelComp/LLR Frame.slot %d.%d ------  \n", frame_rx%1024, nr_slot_rx);
      //to update from pdsch config

      for (uint16_t m=start_symb_sch;m<(nb_symb_sch+start_symb_sch) ; m++){
        nr_slot_fep(ue,
                    proc,
                    m,  //to be updated from higher layer
                    nr_slot_rx);
      }
      VCD_SIGNAL_DUMPER_DUMP_FUNCTION_BY_NAME(VCD_SIGNAL_DUMPER_FUNCTIONS_UE_SLOT_FEP_PDSCH, VCD_FUNCTION_OUT);
    }
  } else {
    LOG_D(PHY,"[UE %d] Frame %d, nr_slot_rx %d: No DCIs found\n", ue->Mod_id, frame_rx, nr_slot_rx);
  }

#endif //NR_PDCCH_SCHED

  // Start PUSCH processing here. It runs in parallel with PDSCH processing
  notifiedFIFO_elt_t *newElt = newNotifiedFIFO_elt(sizeof(nr_rxtx_thread_data_t), proc->nr_slot_tx,txFifo,processSlotTX);
  nr_rxtx_thread_data_t *curMsg=(nr_rxtx_thread_data_t *)NotifiedFifoData(newElt);
  curMsg->proc = *proc;
  curMsg->UE = ue;
  curMsg->ue_sched_mode = ONLY_PUSCH;
  pushTpool(&(get_nrUE_params()->Tpool), newElt);
  start_meas(&ue->generic_stat);
  // do procedures for C-RNTI
  int ret_pdsch = 0;
  if (ue->dlsch[proc->thread_id][gNB_id][0]->active == 1) {
    VCD_SIGNAL_DUMPER_DUMP_FUNCTION_BY_NAME(VCD_SIGNAL_DUMPER_FUNCTIONS_PDSCH_PROC_C, VCD_FUNCTION_IN);
    ret_pdsch = nr_ue_pdsch_procedures(ue,
                                       proc,
                                       gNB_id,
                                       PDSCH,
                                       ue->dlsch[proc->thread_id][gNB_id][0],
                                       NULL);

    nr_ue_measurement_procedures(2, ue, proc, gNB_id, nr_slot_rx);
    VCD_SIGNAL_DUMPER_DUMP_FUNCTION_BY_NAME(VCD_SIGNAL_DUMPER_FUNCTIONS_PDSCH_PROC_C, VCD_FUNCTION_OUT);
  }

  // do procedures for SI-RNTI
  if ((ue->dlsch_SI[gNB_id]) && (ue->dlsch_SI[gNB_id]->active == 1)) {
    VCD_SIGNAL_DUMPER_DUMP_FUNCTION_BY_NAME(VCD_SIGNAL_DUMPER_FUNCTIONS_PDSCH_PROC_SI, VCD_FUNCTION_IN);
    nr_ue_pdsch_procedures(ue,
                           proc,
                           gNB_id,
                           SI_PDSCH,
                           ue->dlsch_SI[gNB_id],
                           NULL);
    
    nr_ue_dlsch_procedures(ue,
                           proc,
                           gNB_id,
                           SI_PDSCH,
                           ue->dlsch_SI[gNB_id],
                           NULL,
                           &ue->dlsch_SI_errors[gNB_id]);

    // deactivate dlsch once dlsch proc is done
    ue->dlsch_SI[gNB_id]->active = 0;

    VCD_SIGNAL_DUMPER_DUMP_FUNCTION_BY_NAME(VCD_SIGNAL_DUMPER_FUNCTIONS_PDSCH_PROC_SI, VCD_FUNCTION_OUT);
  }

  // do procedures for P-RNTI
  if ((ue->dlsch_p[gNB_id]) && (ue->dlsch_p[gNB_id]->active == 1)) {
    VCD_SIGNAL_DUMPER_DUMP_FUNCTION_BY_NAME(VCD_SIGNAL_DUMPER_FUNCTIONS_PDSCH_PROC_P, VCD_FUNCTION_IN);
    nr_ue_pdsch_procedures(ue,
                           proc,
                           gNB_id,
                           P_PDSCH,
                           ue->dlsch_p[gNB_id],
                           NULL);

    nr_ue_dlsch_procedures(ue,
                           proc,
                           gNB_id,
                           P_PDSCH,
                           ue->dlsch_p[gNB_id],
                           NULL,
                           &ue->dlsch_p_errors[gNB_id]);

    // deactivate dlsch once dlsch proc is done
    ue->dlsch_p[gNB_id]->active = 0;
    VCD_SIGNAL_DUMPER_DUMP_FUNCTION_BY_NAME(VCD_SIGNAL_DUMPER_FUNCTIONS_PDSCH_PROC_P, VCD_FUNCTION_OUT);
  }

  // do procedures for RA-RNTI
  if ((ue->dlsch_ra[gNB_id]) && (ue->dlsch_ra[gNB_id]->active == 1) && (ue->UE_mode[gNB_id] != PUSCH)) {
    VCD_SIGNAL_DUMPER_DUMP_FUNCTION_BY_NAME(VCD_SIGNAL_DUMPER_FUNCTIONS_PDSCH_PROC_RA, VCD_FUNCTION_IN);
    nr_ue_pdsch_procedures(ue,
                           proc,
                           gNB_id,
                           RA_PDSCH,
                           ue->dlsch_ra[gNB_id],
                           NULL);

    nr_ue_dlsch_procedures(ue,
                           proc,
                           gNB_id,
                           RA_PDSCH,
                           ue->dlsch_ra[gNB_id],
                           NULL,
                           &ue->dlsch_ra_errors[gNB_id]);

    // deactivate dlsch once dlsch proc is done
    ue->dlsch_ra[gNB_id]->active = 0;

    VCD_SIGNAL_DUMPER_DUMP_FUNCTION_BY_NAME(VCD_SIGNAL_DUMPER_FUNCTIONS_PDSCH_PROC_RA, VCD_FUNCTION_OUT);
  }
  // do procedures for C-RNTI
  if (ue->dlsch[proc->thread_id][gNB_id][0]->active == 1) {

    LOG_D(PHY, "DLSCH data reception at nr_slot_rx: %d\n", nr_slot_rx);
    VCD_SIGNAL_DUMPER_DUMP_FUNCTION_BY_NAME(VCD_SIGNAL_DUMPER_FUNCTIONS_PDSCH_PROC, VCD_FUNCTION_IN);

    start_meas(&ue->dlsch_procedures_stat[proc->thread_id]);

    NR_UE_DLSCH_t *dlsch1 = NULL;
    if (NR_MAX_NB_LAYERS>4)
      dlsch1 = ue->dlsch[proc->thread_id][gNB_id][1];

    if (ret_pdsch >= 0)
      nr_ue_dlsch_procedures(ue,
			   proc,
			   gNB_id,
			   PDSCH,
			   ue->dlsch[proc->thread_id][gNB_id][0],
			   dlsch1,
			   &ue->dlsch_errors[gNB_id]);

  stop_meas(&ue->dlsch_procedures_stat[proc->thread_id]);
  if (cpumeas(CPUMEAS_GETSTATE)) {
    LOG_D(PHY, "[SFN %d] Slot1:       Pdsch Proc %5.2f\n",nr_slot_rx,ue->pdsch_procedures_stat[proc->thread_id].p_time/(cpuf*1000.0));
    LOG_D(PHY, "[SFN %d] Slot0 Slot1: Dlsch Proc %5.2f\n",nr_slot_rx,ue->dlsch_procedures_stat[proc->thread_id].p_time/(cpuf*1000.0));
  }

  // deactivate dlsch once dlsch proc is done
  ue->dlsch[proc->thread_id][gNB_id][0]->active = 0;

  VCD_SIGNAL_DUMPER_DUMP_FUNCTION_BY_NAME(VCD_SIGNAL_DUMPER_FUNCTIONS_PDSCH_PROC, VCD_FUNCTION_OUT);

 }

  // do procedures for CSI-IM
  if ((ue->csiim_vars[gNB_id]) && (ue->csiim_vars[gNB_id]->active == 1)) {
    int l_csiim[4] = {-1, -1, -1, -1};
    for(int symb_idx = 0; symb_idx < 4; symb_idx++) {
      bool nr_slot_fep_done = false;
      for (int symb_idx2 = 0; symb_idx2 < symb_idx; symb_idx2++) {
        if (l_csiim[symb_idx2] == ue->csiim_vars[gNB_id]->csiim_config_pdu.l_csiim[symb_idx]) {
          nr_slot_fep_done = true;
        }
      }
      l_csiim[symb_idx] = ue->csiim_vars[gNB_id]->csiim_config_pdu.l_csiim[symb_idx];
      if(nr_slot_fep_done == false) {
        nr_slot_fep(ue, proc, ue->csiim_vars[gNB_id]->csiim_config_pdu.l_csiim[symb_idx], nr_slot_rx);
      }
    }
    nr_ue_csi_im_procedures(ue, proc, gNB_id);
    ue->csiim_vars[gNB_id]->active = 0;
  }

  // do procedures for CSI-RS
  if ((ue->csirs_vars[gNB_id]) && (ue->csirs_vars[gNB_id]->active == 1)) {
    for(int symb = 0; symb < NR_SYMBOLS_PER_SLOT; symb++) {
      if(is_csi_rs_in_symbol(ue->csirs_vars[gNB_id]->csirs_config_pdu,symb)) {
        nr_slot_fep(ue, proc, symb, nr_slot_rx);
      }
    }
    nr_ue_csi_rs_procedures(ue, proc, gNB_id);
    ue->csirs_vars[gNB_id]->active = 0;
  }

  start_meas(&ue->generic_stat);

  if (nr_slot_rx==9) {
    if (frame_rx % 10 == 0) {
      if ((ue->dlsch_received[gNB_id] - ue->dlsch_received_last[gNB_id]) != 0)
        ue->dlsch_fer[gNB_id] = (100*(ue->dlsch_errors[gNB_id] - ue->dlsch_errors_last[gNB_id]))/(ue->dlsch_received[gNB_id] - ue->dlsch_received_last[gNB_id]);

      ue->dlsch_errors_last[gNB_id] = ue->dlsch_errors[gNB_id];
      ue->dlsch_received_last[gNB_id] = ue->dlsch_received[gNB_id];
    }


    ue->bitrate[gNB_id] = (ue->total_TBS[gNB_id] - ue->total_TBS_last[gNB_id])*100;
    ue->total_TBS_last[gNB_id] = ue->total_TBS[gNB_id];
    LOG_D(PHY,"[UE %d] Calculating bitrate Frame %d: total_TBS = %d, total_TBS_last = %d, bitrate %f kbits\n",
          ue->Mod_id,frame_rx,ue->total_TBS[gNB_id],
          ue->total_TBS_last[gNB_id],(float) ue->bitrate[gNB_id]/1000.0);

#if UE_AUTOTEST_TRACE
    if ((frame_rx % 100 == 0)) {
      LOG_I(PHY,"[UE  %d] AUTOTEST Metric : UE_DLSCH_BITRATE = %5.2f kbps (frame = %d) \n", ue->Mod_id, (float) ue->bitrate[gNB_id]/1000.0, frame_rx);
    }
#endif

  }

  stop_meas(&ue->generic_stat);
  if (cpumeas(CPUMEAS_GETSTATE))
    LOG_D(PHY,"after tubo until end of Rx %5.2f \n",ue->generic_stat.p_time/(cpuf*1000.0));

#ifdef EMOS
  phy_procedures_emos_UE_RX(ue,slot,gNB_id);
#endif


  VCD_SIGNAL_DUMPER_DUMP_FUNCTION_BY_NAME(VCD_SIGNAL_DUMPER_FUNCTIONS_PHY_PROCEDURES_UE_RX, VCD_FUNCTION_OUT);

  stop_meas(&ue->phy_proc_rx[proc->thread_id]);
  if (cpumeas(CPUMEAS_GETSTATE))
    LOG_D(PHY, "------FULL RX PROC [SFN %d]: %5.2f ------\n",nr_slot_rx,ue->phy_proc_rx[proc->thread_id].p_time/(cpuf*1000.0));

  LOG_D(PHY," ****** end RX-Chain  for AbsSubframe %d.%d ******  \n", frame_rx%1024, nr_slot_rx);
  return (0);
}


uint8_t nr_is_cqi_TXOp(PHY_VARS_NR_UE *ue,
		            UE_nr_rxtx_proc_t *proc,
					uint8_t gNB_id)
{
  int subframe = proc->nr_slot_tx / ue->frame_parms.slots_per_subframe;
  int frame    = proc->frame_tx;
  CQI_REPORTPERIODIC *cqirep = &ue->cqi_report_config[gNB_id].CQI_ReportPeriodic;

  //LOG_I(PHY,"[UE %d][CRNTI %x] AbsSubFrame %d.%d Checking for CQI TXOp (cqi_ConfigIndex %d) isCQIOp %d\n",
  //      ue->Mod_id,ue->pdcch_vars[gNB_id]->crnti,frame,subframe,
  //      cqirep->cqi_PMI_ConfigIndex,
  //      (((10*frame + subframe) % cqirep->Npd) == cqirep->N_OFFSET_CQI));

  if (cqirep->cqi_PMI_ConfigIndex==-1)
    return(0);
  else if (((10*frame + subframe) % cqirep->Npd) == cqirep->N_OFFSET_CQI)
    return(1);
  else
    return(0);
}


uint8_t nr_is_ri_TXOp(PHY_VARS_NR_UE *ue,
		           UE_nr_rxtx_proc_t *proc,
				   uint8_t gNB_id)
{
  int subframe = proc->nr_slot_tx / ue->frame_parms.slots_per_subframe;
  int frame    = proc->frame_tx;
  CQI_REPORTPERIODIC *cqirep = &ue->cqi_report_config[gNB_id].CQI_ReportPeriodic;
  int log2Mri = cqirep->ri_ConfigIndex/161;
  int N_OFFSET_RI = cqirep->ri_ConfigIndex % 161;

  //LOG_I(PHY,"[UE %d][CRNTI %x] AbsSubFrame %d.%d Checking for RI TXOp (ri_ConfigIndex %d) isRIOp %d\n",
  //      ue->Mod_id,ue->pdcch_vars[gNB_id]->crnti,frame,subframe,
  //      cqirep->ri_ConfigIndex,
  //      (((10*frame + subframe + cqirep->N_OFFSET_CQI - N_OFFSET_RI) % (cqirep->Npd<<log2Mri)) == 0));
  if (cqirep->ri_ConfigIndex==-1)
    return(0);
  else if (((10*frame + subframe + cqirep->N_OFFSET_CQI - N_OFFSET_RI) % (cqirep->Npd<<log2Mri)) == 0)
    return(1);
  else
    return(0);
}

// todo:
// - power control as per 38.213 ch 7.4
void nr_ue_prach_procedures(PHY_VARS_NR_UE *ue, UE_nr_rxtx_proc_t *proc, uint8_t gNB_id) {

  int frame_tx = proc->frame_tx, nr_slot_tx = proc->nr_slot_tx, prach_power; // tx_amp
  uint8_t mod_id = ue->Mod_id;
  NR_PRACH_RESOURCES_t * prach_resources = ue->prach_resources[gNB_id];
  AssertFatal(prach_resources != NULL, "ue->prach_resources[%u] == NULL\n", gNB_id);
  uint8_t nr_prach = 0;

  VCD_SIGNAL_DUMPER_DUMP_FUNCTION_BY_NAME(VCD_SIGNAL_DUMPER_FUNCTIONS_PHY_PROCEDURES_UE_TX_PRACH, VCD_FUNCTION_IN);

  if (ue->mac_enabled == 0){

    prach_resources->ra_TDD_map_index = 0;
    prach_resources->ra_PREAMBLE_RECEIVED_TARGET_POWER = 10;
    prach_resources->ra_RNTI = 0x1234;
    nr_prach = 1;
    prach_resources->init_msg1 = 1;

  } else {

    nr_prach = nr_ue_get_rach(prach_resources, &ue->prach_vars[0]->prach_pdu, mod_id, ue->CC_id, frame_tx, gNB_id, nr_slot_tx);
    LOG_D(PHY, "In %s:[%d.%d] getting PRACH resources : %d\n", __FUNCTION__, frame_tx, nr_slot_tx,nr_prach);
  }

  if (nr_prach == GENERATE_PREAMBLE) {

    if (ue->mac_enabled == 1) {
      int16_t pathloss = get_nr_PL(mod_id, ue->CC_id, gNB_id);
      int16_t ra_preamble_rx_power = (int16_t)(prach_resources->ra_PREAMBLE_RECEIVED_TARGET_POWER - pathloss + 30);
      ue->tx_power_dBm[nr_slot_tx] = min(nr_get_Pcmax(mod_id), ra_preamble_rx_power);

      LOG_D(PHY, "In %s: [UE %d][RAPROC][%d.%d]: Generating PRACH Msg1 (preamble %d, PL %d dB, P0_PRACH %d, TARGET_RECEIVED_POWER %d dBm, RA-RNTI %x)\n",
        __FUNCTION__,
        mod_id,
        frame_tx,
        nr_slot_tx,
        prach_resources->ra_PreambleIndex,
        pathloss,
        ue->tx_power_dBm[nr_slot_tx],
        prach_resources->ra_PREAMBLE_RECEIVED_TARGET_POWER,
        prach_resources->ra_RNTI);
    }

    ue->prach_vars[gNB_id]->amp = AMP;

    /* #if defined(EXMIMO) || defined(OAI_USRP) || defined(OAI_BLADERF) || defined(OAI_LMSSDR) || defined(OAI_ADRV9371_ZC706)
      tx_amp = get_tx_amp_prach(ue->tx_power_dBm[nr_slot_tx], ue->tx_power_max_dBm, ue->frame_parms.N_RB_UL);
      if (tx_amp != -1)
        ue->prach_vars[gNB_id]->amp = tx_amp;
    #else
      ue->prach_vars[gNB_id]->amp = AMP;
    #endif */

    VCD_SIGNAL_DUMPER_DUMP_FUNCTION_BY_NAME(VCD_SIGNAL_DUMPER_FUNCTIONS_UE_GENERATE_PRACH, VCD_FUNCTION_IN);

    prach_power = generate_nr_prach(ue, gNB_id, frame_tx, nr_slot_tx);

    VCD_SIGNAL_DUMPER_DUMP_FUNCTION_BY_NAME(VCD_SIGNAL_DUMPER_FUNCTIONS_UE_GENERATE_PRACH, VCD_FUNCTION_OUT);

    LOG_D(PHY, "In %s: [UE %d][RAPROC][%d.%d]: Generated PRACH Msg1 (TX power PRACH %d dBm, digital power %d dBW (amp %d)\n",
      __FUNCTION__,
      mod_id,
      frame_tx,
      nr_slot_tx,
      ue->tx_power_dBm[nr_slot_tx],
      dB_fixed(prach_power),
      ue->prach_vars[gNB_id]->amp);

    if (ue->mac_enabled == 1)
      nr_Msg1_transmitted(mod_id, ue->CC_id, frame_tx, gNB_id);

  } else if (nr_prach == WAIT_CONTENTION_RESOLUTION) {
    LOG_D(PHY, "In %s: [UE %d] RA waiting contention resolution\n", __FUNCTION__, mod_id);
    ue->UE_mode[gNB_id] = RA_WAIT_CR;
  } else if (nr_prach == RA_SUCCEEDED) {
    LOG_D(PHY, "In %s: [UE %d] RA completed, setting UE mode to PUSCH\n", __FUNCTION__, mod_id);
    ue->UE_mode[gNB_id] = PUSCH;
  } else if(nr_prach == RA_FAILED){
    LOG_D(PHY, "In %s: [UE %d] RA failed, setting UE mode to PRACH\n", __FUNCTION__, mod_id);
    ue->UE_mode[gNB_id] = PRACH;
  }

  VCD_SIGNAL_DUMPER_DUMP_FUNCTION_BY_NAME(VCD_SIGNAL_DUMPER_FUNCTIONS_PHY_PROCEDURES_UE_TX_PRACH, VCD_FUNCTION_OUT);

}
