/*
 * Licensed to the OpenAirInterface (OAI) Software Alliance under one or more
 * contributor license agreements.  See the NOTICE file distributed with
 * this work for additional information regarding copyright ownership.
 * The OpenAirInterface Software Alliance licenses this file to You under
 * the OAI Public License, Version 1.0  (the "License"); you may not use this file
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

/*! \file PHY/NR_UE_TRANSPORT/nr_dlsch_decoding.c
* \brief Top-level routines for decoding  Turbo-coded (DLSCH) transport channels from 36-212, V8.6 2009-03
* \author R. Knopp
* \date 2011
* \version 0.1
* \company Eurecom
* \email: knopp@eurecom.fr
* \note
* \warning
*/

// [from nrUE coding]
// #include "PHY/CODING/lte_interleaver_inline.h"
// #include "PHY/NR_UE_TRANSPORT/nr_transport_ue.h"
// #include "common/utils/LOG/vcd_signal_dumper.h"
// #include <openair2/UTIL/OPT/opt.h>

#include "common/utils/LOG/vcd_signal_dumper.h"
#include "PHY/defs_nr_UE.h"
#include "SCHED_NR_UE/harq_nr.h"
#include "PHY/phy_extern_nr_ue.h"
#include "PHY/CODING/coding_extern.h"
#include "PHY/CODING/coding_defs.h"
#include "PHY/NR_TRANSPORT/nr_transport_common_proto.h"
#include "PHY/NR_UE_TRANSPORT/nr_transport_proto_ue.h"
#include "PHY/NR_TRANSPORT/nr_dlsch.h"
#include "SCHED_NR_UE/defs.h"
#include "SIMULATION/TOOLS/sim.h"
#include "executables/nr-uesoftmodem.h"
#include "PHY/CODING/nrLDPC_extern.h"
#include "common/utils/nr/nr_common.h"

//#define ENABLE_PHY_PAYLOAD_DEBUG 1
#define NR_POLAR_SCI2_MESSAGE_TYPE 4
#define NR_POLAR_SCI2_PAYLOAD_BITS 35
#define NR_POLAR_SCI2_AGGREGATION_LEVEL 0

#define OAI_UL_LDPC_MAX_NUM_LLR 27000//26112 // NR_LDPC_NCOL_BG1*NR_LDPC_ZMAX = 68*384
//#define OAI_LDPC_MAX_NUM_LLR 27000//26112 // NR_LDPC_NCOL_BG1*NR_LDPC_ZMAX

static uint64_t nb_total_decod =0;
static uint64_t nb_error_decod =0;

// notifiedFIFO_t freeBlocks_dl;
// notifiedFIFO_elt_t *msgToPush_dl;
//int nbDlProcessing =0; 

static void nr_sci2_quantize(int16_t *psbch_llr8,
                             int16_t *psbch_llr,
                             uint16_t len) {
  for (int i = 0; i < len; i++) {
    if (psbch_llr[i] > 31)
      psbch_llr8[i] = 32;
    else if (psbch_llr[i] < -31)
      psbch_llr8[i] = -32;
    else
      psbch_llr8[i] = psbch_llr[i];
  }
}

bool nr_ue_sl_postDecode(PHY_VARS_NR_UE *phy_vars_ue, notifiedFIFO_elt_t *req, bool last, notifiedFIFO_t *nf_p) {
  ldpcDecode_ue_t *rdata = (ldpcDecode_ue_t*) NotifiedFifoData(req);
  NR_DL_UE_HARQ_t *harq_process = rdata->harq_process;
  NR_UE_DLSCH_t *dlsch = (NR_UE_DLSCH_t *) rdata->dlsch;
  int r = rdata->segment_r;

  merge_meas(&phy_vars_ue->dlsch_deinterleaving_stats, &rdata->ts_deinterleave);
  merge_meas(&phy_vars_ue->dlsch_rate_unmatching_stats, &rdata->ts_rate_unmatch);
  merge_meas(&phy_vars_ue->dlsch_ldpc_decoding_stats, &rdata->ts_ldpc_decode);

  bool decodeSuccess = (rdata->decodeIterations < (1+dlsch->max_ldpc_iterations));
  LOG_D(PHY, "rdata->decodeIterations %d\n", rdata->decodeIterations);

  if (1) {
    memcpy(harq_process->b+rdata->offset,
           harq_process->c[r],
           rdata->Kr_bytes - (harq_process->F>>3) -((harq_process->C>1)?3:0));

  } else {
    if ( !last ) {
      int nb=abortTpoolJob(&get_nrUE_params()->Tpool, req->key);
      nb+=abortNotifiedFIFOJob(nf_p, req->key);
      LOG_D(PHY,"downlink segment error %d/%d, aborted %d segments\n",rdata->segment_r,rdata->nbSegments, nb);
      LOG_D(PHY, "SLSCH %d in error\n",rdata->dlsch_id);
      last = true;
    }
  }

  // if all segments are done
  if (last) {
    if (decodeSuccess) {
      //LOG_D(PHY,"[UE %d] DLSCH: Setting ACK for nr_slot_rx %d TBS %d mcs %d nb_rb %d harq_process->round %d\n",
      //      phy_vars_ue->Mod_id,nr_slot_rx,harq_process->TBS,harq_process->mcs,harq_process->nb_rb, harq_process->round);
      harq_process->status = SCH_IDLE;
      harq_process->ack = 1;

      //LOG_D(PHY,"[UE %d] DLSCH: Setting ACK for SFN/SF %d/%d (pid %d, status %d, round %d, TBS %d, mcs %d)\n",
      //  phy_vars_ue->Mod_id, frame, subframe, harq_pid, harq_process->status, harq_process->round,harq_process->TBS,harq_process->mcs);

      //if(is_crnti) {
      //  LOG_D(PHY,"[UE %d] DLSCH: Setting ACK for nr_slot_rx %d (pid %d, round %d, TBS %d)\n",phy_vars_ue->Mod_id,nr_slot_rx,harq_pid,harq_process->round,harq_process->TBS);
      //}
      dlsch->last_iteration_cnt = rdata->decodeIterations;
      LOG_I(PHY, "SLSCH received ok \n");
    } else {
      //LOG_D(PHY,"[UE %d] DLSCH: Setting NAK for SFN/SF %d/%d (pid %d, status %d, round %d, TBS %d, mcs %d) Kr %d r %d harq_process->round %d\n",
      //      phy_vars_ue->Mod_id, frame, nr_slot_rx, harq_pid,harq_process->status, harq_process->round,harq_process->TBS,harq_process->mcs,Kr,r,harq_process->round);
      harq_process->ack = 0;

      //if(is_crnti) {
      //  LOG_D(PHY,"[UE %d] DLSCH: Setting NACK for nr_slot_rx %d (pid %d, pid status %d, round %d/Max %d, TBS %d)\n",
      //        phy_vars_ue->Mod_id,nr_slot_rx,harq_pid,harq_process->status,harq_process->round,dlsch->Mdlharq,harq_process->TBS);
      //}
      dlsch->last_iteration_cnt = dlsch->max_ldpc_iterations + 1;
      LOG_I(PHY, "SLSCH received nok \n");
    }
    return true; //stop
  }
  else
  {
	return false; //not last one
  }
}

void nr_sl_processDLSegment(void* arg) {
  ldpcDecode_ue_t *rdata = (ldpcDecode_ue_t*) arg;
  NR_UE_DLSCH_t *dlsch = rdata->dlsch;
  NR_DL_UE_HARQ_t *harq_process= rdata->harq_process;
  t_nrLDPC_dec_params *p_decoderParms = &rdata->decoderParms;
  int length_dec;
  int no_iteration_ldpc;
  int Kr;
  int K_bits_F;
  uint8_t crc_type;
  int i;
  int j;
  int r = rdata->segment_r;
  int A = rdata->A;
  int E = rdata->E;
  int Qm = rdata->Qm;
  //int rv_index = rdata->rv_index;
  int r_offset = rdata->r_offset;
  uint8_t kc = rdata->Kc;
  uint32_t Tbslbrm = rdata->Tbslbrm;
  short* dlsch_llr = rdata->dlsch_llr;
  rdata->decodeIterations = dlsch->max_ldpc_iterations + 1;
  int8_t llrProcBuf[OAI_UL_LDPC_MAX_NUM_LLR] __attribute__ ((aligned(32)));

  int16_t  z [68*384 + 16] __attribute__ ((aligned(16)));
  int8_t   l [68*384 + 16] __attribute__ ((aligned(16)));

  __m128i *pv = (__m128i*)&z;
  __m128i *pl = (__m128i*)&l;

  Kr = harq_process->K;
  K_bits_F = Kr-harq_process->F;

  t_nrLDPC_time_stats procTime = {0};
  t_nrLDPC_time_stats* p_procTime = &procTime ;

  start_meas(&rdata->ts_deinterleave);

  //VCD_SIGNAL_DUMPER_DUMP_FUNCTION_BY_NAME(VCD_SIGNAL_DUMPER_FUNCTIONS_DLSCH_DEINTERLEAVING, VCD_FUNCTION_IN);
  int16_t harq_e[E];
  nr_deinterleaving_ldpc(E,
                         Qm,
                         harq_e, // [hna] w is e
                         dlsch_llr+r_offset);
  //VCD_SIGNAL_DUMPER_DUMP_FUNCTION_BY_NAME(VCD_SIGNAL_DUMPER_FUNCTIONS_DLSCH_DEINTERLEAVING, VCD_FUNCTION_OUT);
  stop_meas(&rdata->ts_deinterleave);

  start_meas(&rdata->ts_rate_unmatch);
  /* LOG_D(PHY,"HARQ_PID %d Rate Matching Segment %d (coded bits %d,E %d, F %d,unpunctured/repeated bits %d, TBS %d, mod_order %d, nb_rb %d, Nl %d, rv %d, round %d)...\n",
        harq_pid,r, G,E,harq_process->F,
        Kr*3,
        harq_process->TBS,
        Qm,
        harq_process->nb_rb,
        harq_process->Nl,
        harq_process->rvidx,
        harq_process->round); */
  //VCD_SIGNAL_DUMPER_DUMP_FUNCTION_BY_NAME(VCD_SIGNAL_DUMPER_FUNCTIONS_DLSCH_RATE_MATCHING, VCD_FUNCTION_IN);

  if (nr_rate_matching_ldpc_rx(Tbslbrm,
                               p_decoderParms->BG,
                               p_decoderParms->Z,
                               harq_process->d[r],
                               harq_e,
                               harq_process->C,
                               harq_process->rvidx,
                               harq_process->first_rx,
                               E,
                               harq_process->F,
                               Kr - harq_process->F - 2 * (p_decoderParms->Z), true) == -1) {
    //VCD_SIGNAL_DUMPER_DUMP_FUNCTION_BY_NAME(VCD_SIGNAL_DUMPER_FUNCTIONS_DLSCH_RATE_MATCHING, VCD_FUNCTION_OUT);
    stop_meas(&rdata->ts_rate_unmatch);
    LOG_E(PHY,"dlsch_decoding.c: Problem in rate_matching\n");
    rdata->decodeIterations = dlsch->max_ldpc_iterations + 1;
    return;
  }
  stop_meas(&rdata->ts_rate_unmatch);

  if (LOG_DEBUGFLAG(DEBUG_DLSCH_DECOD)) {
    LOG_D(PHY,"decoder input(segment %u) :",r);

    for (int i=0; i<E; i++)
      LOG_D(PHY,"%d : %d\n",i,harq_process->d[r][i]);

    LOG_D(PHY,"\n");
  }

  if (harq_process->C == 1) {
    if (A > NR_MAX_PDSCH_TBS)
      crc_type = CRC24_A;
    else
      crc_type = CRC16;

    length_dec = harq_process->B;
  } else {
    crc_type = CRC24_B;
    length_dec = (harq_process->B+24*harq_process->C)/harq_process->C;
  }

  {
    start_meas(&rdata->ts_ldpc_decode);
    //set first 2*Z_c bits to zeros
    memset(&z[0],0,2*harq_process->Z*sizeof(int16_t));
    //set Filler bits
    memset((&z[0]+K_bits_F),127,harq_process->F*sizeof(int16_t));
    //Move coded bits before filler bits
    memcpy((&z[0]+2*harq_process->Z),harq_process->d[r],(K_bits_F-2*harq_process->Z)*sizeof(int16_t));
    //skip filler bits
    memcpy((&z[0]+Kr),harq_process->d[r]+(Kr-2*harq_process->Z),(kc*harq_process->Z-Kr)*sizeof(int16_t));

    //Saturate coded bits before decoding into 8 bits values
    for (i=0, j=0; j < ((kc*harq_process->Z)>>4)+1;  i+=2, j++) {
      pl[j] = _mm_packs_epi16(pv[i],pv[i+1]);
    }

    //VCD_SIGNAL_DUMPER_DUMP_FUNCTION_BY_NAME(VCD_SIGNAL_DUMPER_FUNCTIONS_DLSCH_LDPC, VCD_FUNCTION_IN);
    p_decoderParms->block_length=length_dec;
    nrLDPC_initcall(p_decoderParms, (int8_t*)&pl[0], llrProcBuf);
    no_iteration_ldpc = nrLDPC_decoder(p_decoderParms,
                                       (int8_t *)&pl[0],
                                       llrProcBuf,
                                       p_procTime);
    //VCD_SIGNAL_DUMPER_DUMP_FUNCTION_BY_NAME(VCD_SIGNAL_DUMPER_FUNCTIONS_DLSCH_LDPC, VCD_FUNCTION_OUT);

    LOG_D(PHY,"no_iteration_ldpc = %d\n", no_iteration_ldpc);
    // Fixme: correct type is unsigned, but nrLDPC_decoder and all called behind use signed int
    if (check_crc((uint8_t *)llrProcBuf,length_dec,harq_process->F,crc_type)) {
      LOG_D(PHY,"Segment %u CRC OK\n",r);
      if (no_iteration_ldpc > dlsch->max_ldpc_iterations)
        no_iteration_ldpc = dlsch->max_ldpc_iterations;
    } else {
      LOG_D(PHY,"CRC NOT OK\n");
      no_iteration_ldpc = dlsch->max_ldpc_iterations + 1;
    }

    if (r==0) {
      for (int i=0; i<10; i++) LOG_D(PHY,"byte %d : %x\n",i,((uint8_t *)llrProcBuf)[i]);
    }

    rdata->decodeIterations = no_iteration_ldpc;

    nb_total_decod++;

    if (no_iteration_ldpc > dlsch->max_ldpc_iterations) {
      nb_error_decod++;
    }

    for (int m=0; m < Kr>>3; m ++) {
      harq_process->c[r][m]= (uint8_t) llrProcBuf[m];
    }

    stop_meas(&rdata->ts_ldpc_decode);
  }
}

uint32_t nr_slsch_decoding(PHY_VARS_NR_UE *phy_vars_ue,
                           UE_nr_rxtx_proc_t *proc,
                           short *dlsch_llr,
                           NR_DL_FRAME_PARMS *frame_parms,
                           NR_UE_DLSCH_t *dlsch,
                           NR_DL_UE_HARQ_t *harq_process,
                           uint32_t frame,
                           uint16_t nb_symb_sch,
                           uint8_t nr_slot_rx,
                           uint8_t harq_pid) {
  uint32_t A,E;
  uint32_t G;
  uint32_t ret,offset;
  uint32_t r,r_offset=0,Kr=8424,Kr_bytes;
  t_nrLDPC_dec_params decParams;
  t_nrLDPC_dec_params *p_decParams = &decParams;

  if (!harq_process) {
    LOG_E(PHY,"slsch_decoding.c: NULL harq_process pointer\n");
    return(dlsch->max_ldpc_iterations + 1);
  }

  // HARQ stats
  phy_vars_ue->dl_stats[harq_process->DLround]++;
  LOG_D(PHY,"Round %d RV idx %d\n",harq_process->DLround,harq_process->rvidx);
  uint8_t kc;
  uint16_t nb_rb;// = 30;
  uint8_t dmrs_Type = harq_process->dmrsConfigType;
  AssertFatal(dmrs_Type == 0 || dmrs_Type == 1, "Illegal dmrs_type %d\n", dmrs_Type);
  uint8_t nb_re_dmrs;

  if (dmrs_Type==NFAPI_NR_DMRS_TYPE1) {
    nb_re_dmrs = 6*harq_process->n_dmrs_cdm_groups;
  } else {
    nb_re_dmrs = 4*harq_process->n_dmrs_cdm_groups;
  }

  uint16_t dmrs_length = get_num_dmrs(harq_process->dlDmrsSymbPos);
  vcd_signal_dumper_dump_function_by_name(VCD_SIGNAL_DUMPER_FUNCTIONS_DLSCH_SEGMENTATION, VCD_FUNCTION_IN);

  //NR_DL_UE_HARQ_t *harq_process = dlsch->harq_processes[0];

  int nbDecode = 0;

  if (!dlsch_llr) {
    LOG_E(PHY,"slsch_decoding.c: NULL dlsch_llr pointer\n");
    return(dlsch->max_ldpc_iterations + 1);
  }

  if (!frame_parms) {
    LOG_E(PHY,"slsch_decoding.c: NULL frame_parms pointer\n");
    return(dlsch->max_ldpc_iterations + 1);
  }

  // SCI2 decoding //
  short *sci2_llr = dlsch_llr;
  short *data_llr = dlsch_llr + harq_process->B_sci2 * harq_process->Nl;

  #if 1
  int16_t decoder_input[1792] = {0}; // 1792 = harq_process->B_sci2
  nr_sci2_quantize(decoder_input, sci2_llr, harq_process->B_sci2);
  uint32_t decoder_state = polar_decoder_int16(decoder_input,
                                              harq_process->b_sci2,
                                              0,
                                              NR_POLAR_SCI2_MESSAGE_TYPE,
                                              harq_process->B_sci2,
                                              NR_POLAR_SCI2_AGGREGATION_LEVEL);

  if (decoder_state) {
    LOG_E(NR_PHY, "polar_decoder_int16 failed! with ret %d for slot %d\n", decoder_state, nr_slot_rx);
    return(decoder_state);
  }
  #endif

  nb_rb = harq_process->nb_rb;
  A = (harq_process->TBS) << 3;
  ret = dlsch->max_ldpc_iterations + 1;
  dlsch->last_iteration_cnt = ret;
  G = harq_process->G;

  // target_code_rate is in 0.1 units
  float Coderate = (float) harq_process->R / 10240.0f;

  LOG_D(PHY,"%d.%d DLSCH Decoding, harq_pid %d TBS %d (%d) G %d nb_re_dmrs %d length dmrs %d mcs %d Nl %d nb_symb_sch %d nb_rb %d Qm %d Coderate %f\n",
        frame,nr_slot_rx,harq_pid,A,A/8,G, nb_re_dmrs, dmrs_length, harq_process->mcs, harq_process->Nl, nb_symb_sch, nb_rb, harq_process->Qm, Coderate);

  if ((A <=292) || ((A <= NR_MAX_PDSCH_TBS) && (Coderate <= 0.6667)) || Coderate <= 0.25) {
    p_decParams->BG = 2;
    kc = 52;

    if (Coderate < 0.3333) {
      p_decParams->R = 15;
    } else if (Coderate <0.6667) {
      p_decParams->R = 13;
    } else {
      p_decParams->R = 23;
    }
  } else {
    p_decParams->BG = 1;
    kc = 68;

    if (Coderate < 0.6667) {
      p_decParams->R = 13;
    } else if (Coderate <0.8889) {
      p_decParams->R = 23;
    } else {
      p_decParams->R = 89;
    }
  }

  if (harq_process->first_rx == 1) {
    // This is a new packet, so compute quantities regarding segmentation
    if (A > NR_MAX_PDSCH_TBS)
      harq_process->B = A+24;
    else
      harq_process->B = A+16;

    nr_segmentation(NULL,
                    NULL,
                    harq_process->B,
                    &harq_process->C,
                    &harq_process->K,
                    &harq_process->Z, // [hna] Z is Zc
                    &harq_process->F,
                    p_decParams->BG);

    if (harq_process->C>MAX_NUM_NR_DLSCH_SEGMENTS_PER_LAYER*harq_process->Nl) {
      LOG_E(PHY,"nr_segmentation.c: too many segments %d, B %d\n",harq_process->C,harq_process->B);
      return(-1);
    }

    if (LOG_DEBUGFLAG(DEBUG_DLSCH_DECOD) && (!frame%100))
      LOG_I(PHY,"K %d C %d Z %d nl %d \n", harq_process->K, harq_process->C, harq_process->Z, harq_process->Nl);
  }

  VCD_SIGNAL_DUMPER_DUMP_FUNCTION_BY_NAME(VCD_SIGNAL_DUMPER_FUNCTIONS_DLSCH_SEGMENTATION, VCD_FUNCTION_OUT);
  p_decParams->Z = harq_process->Z;
  //printf("dlsch decoding nr segmentation Z %d\n", p_decParams->Z);
  //printf("coderate %f kc %d \n", Coderate, kc);
  p_decParams->numMaxIter = dlsch->max_ldpc_iterations;
  p_decParams->outMode= 0;
  r_offset = 0;
  uint16_t a_segments = MAX_NUM_NR_DLSCH_SEGMENTS_PER_LAYER*harq_process->Nl;  //number of segments to be allocated

  if (nb_rb != 273) {
    a_segments = a_segments*nb_rb;
    a_segments = a_segments/273 +1;
  }

  if (harq_process->C > a_segments) {
    LOG_E(PHY,"Illegal harq_process->C %d > %d\n",harq_process->C,a_segments);
    return((1+dlsch->max_ldpc_iterations));
  }

  if (LOG_DEBUGFLAG(DEBUG_DLSCH_DECOD))
    LOG_I(PHY,"Segmentation: C %d, K %d\n",harq_process->C,harq_process->K);

  Kr = harq_process->K; // [hna] overwrites this line "Kr = p_decParams->Z*kb"
  Kr_bytes = Kr>>3;
  offset = 0;
  void (*nr_processDLSegment_ptr)(void*) = &nr_sl_processDLSegment;
  for (r=0; r<harq_process->C; r++) {
    //printf("start rx segment %d\n",r);
    E = nr_get_E(G, harq_process->C, harq_process->Qm, harq_process->Nl, r);
    union ldpcReqUnion id = {.s={dlsch->rnti,frame,nr_slot_rx,0,0}};
    notifiedFIFO_elt_t *req=newNotifiedFIFO_elt(sizeof(ldpcDecode_ue_t), id.p, &phy_vars_ue->respDecode, nr_processDLSegment_ptr);
    ldpcDecode_ue_t * rdata=(ldpcDecode_ue_t *) NotifiedFifoData(req);

    rdata->phy_vars_ue = phy_vars_ue;
    rdata->harq_process = harq_process;
    rdata->decoderParms = decParams;
    rdata->dlsch_llr = data_llr;
    rdata->Kc = kc;
    rdata->harq_pid = harq_pid;
    rdata->segment_r = r;
    rdata->nbSegments = harq_process->C;
    rdata->E = E;
    rdata->A = A;
    rdata->Qm = harq_process->Qm;
    rdata->r_offset = r_offset;
    rdata->Kr_bytes = Kr_bytes;
    rdata->rv_index = harq_process->rvidx;
    rdata->Tbslbrm = harq_process->tbslbrm;
    rdata->offset = offset;
    rdata->dlsch = dlsch;
    rdata->dlsch_id = 0;
    reset_meas(&rdata->ts_deinterleave);
    reset_meas(&rdata->ts_rate_unmatch);
    reset_meas(&rdata->ts_ldpc_decode);
    pushTpool(&get_nrUE_params()->Tpool,req);
    nbDecode++;
    LOG_D(PHY,"Added a block to decode, in pipe: %d\n",nbDecode);
    r_offset += E;
    offset += (Kr_bytes - (harq_process->F>>3) - ((harq_process->C>1)?3:0));
    //////////////////////////////////////////////////////////////////////////////////////////
  }
  for (r=0; r<nbDecode; r++) {
    notifiedFIFO_elt_t *req=pullTpool(&phy_vars_ue->respDecode, &phy_vars_ue->threadPool);
    if (req == NULL)
      break; // Tpool has been stopped
    bool last = false;
    if (r == nbDecode - 1)
      last = true;
    bool stop = nr_ue_sl_postDecode(phy_vars_ue, req, last, &phy_vars_ue->respDecode);
    delNotifiedFIFO_elt(req);
    if (stop)
      break;
  }

  VCD_SIGNAL_DUMPER_DUMP_FUNCTION_BY_NAME(VCD_SIGNAL_DUMPER_FUNCTIONS_DLSCH_COMBINE_SEG, VCD_FUNCTION_OUT);
  ret = dlsch->last_iteration_cnt;
  return(ret);
}
