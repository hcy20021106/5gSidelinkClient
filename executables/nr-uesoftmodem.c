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


#define _GNU_SOURCE             /* See feature_test_macros(7) */
#include <sched.h>

#include "T.h"
#include "assertions.h"
#include "PHY/types.h"
#include "PHY/defs_nr_UE.h"
#include "SCHED_NR_UE/defs.h"
#include "common/ran_context.h"
#include "common/config/config_userapi.h"
//#include "common/utils/threadPool/thread-pool.h"
#include "common/utils/load_module_shlib.h"
//#undef FRAME_LENGTH_COMPLEX_SAMPLES //there are two conflicting definitions, so we better make sure we don't use it at all
#include "common/utils/nr/nr_common.h"

#include "sdr/COMMON/common_lib.h"
#include "sdr/ETHERNET/USERSPACE/LIB/if_defs.h"

//#undef FRAME_LENGTH_COMPLEX_SAMPLES //there are two conflicting definitions, so we better make sure we don't use it at all
#include "openair1/PHY/MODULATION/nr_modulation.h"
#include "PHY/phy_vars_nr_ue.h"
#include "PHY/NR_UE_TRANSPORT/nr_transport_proto_ue.h"
#include "SCHED/sched_common_vars.h"
#include "PHY/MODULATION/modulation_vars.h"
#include "PHY/NR_TRANSPORT/nr_dlsch.h"
//#include "../../SIMU/USER/init_lte.h"

#include "LAYER2/MAC/mac_vars.h"
#include "RRC/LTE/rrc_vars.h"
#include "PHY_INTERFACE/phy_interface_vars.h"
#include "NR_IF_Module.h"
#include "openair1/SIMULATION/TOOLS/sim.h"

#ifdef SMBV
#include "PHY/TOOLS/smbv.h"
unsigned short config_frames[4] = {2,9,11,13};
#endif
#include "common/utils/LOG/log.h"
#include "common/utils/LOG/vcd_signal_dumper.h"

#include "UTIL/OPT/opt.h"
#include "enb_config.h"

#include "intertask_interface.h"

#include "PHY/INIT/phy_init.h"
#include "system.h"
#include <openair2/RRC/NR_UE/rrc_proto.h>
#include <openair2/LAYER2/NR_MAC_UE/mac_defs.h>
#include <openair2/LAYER2/NR_MAC_UE/mac_proto.h>
#include <openair2/NR_UE_PHY_INTERFACE/NR_IF_Module.h>
#include <openair1/SCHED_NR_UE/fapi_nr_ue_l1.h>

/* Callbacks, globals and object handlers */

//#include "stats.h"
// current status is that every UE has a DL scope for a SINGLE eNB (eNB_id=0)
#include "PHY/TOOLS/phy_scope_interface.h"
#include "PHY/TOOLS/nr_phy_scope.h"
#include <executables/nr-uesoftmodem.h>
#include "executables/softmodem-common.h"
#include "executables/thread-common.h"

#include "nr_nas_msg_sim.h"
#include <openair1/PHY/MODULATION/nr_modulation.h>
#include <openair1/PHY/NR_REFSIG/sss_nr.h>

extern const char *duplex_mode[];
THREAD_STRUCT thread_struct;
nrUE_params_t nrUE_params;

int udp_socket = -1;

// Thread variables
pthread_cond_t nfapi_sync_cond;
pthread_mutex_t nfapi_sync_mutex;
int nfapi_sync_var=-1; //!< protected by mutex \ref nfapi_sync_mutex
uint16_t sf_ahead=6; //??? value ???
pthread_cond_t sync_cond;
pthread_mutex_t sync_mutex;
int sync_var=-1; //!< protected by mutex \ref sync_mutex.
int config_sync_var=-1;

// not used in UE
instance_t CUuniqInstance=0;
instance_t DUuniqInstance=0;

RAN_CONTEXT_t RC;
int oai_exit = 0;


extern int16_t  nr_dlsch_demod_shift;
static int      tx_max_power[MAX_NUM_CCs] = {0};

int      single_thread_flag = 1;
int                 tddflag = 0;
int                 vcdflag = 0;

double          rx_gain_off = 0.0;
char             *usrp_args = NULL;
char       *rrc_config_path = NULL;
char            *uecap_file = NULL;
int               dumpframe = 0;

uint64_t        downlink_frequency[MAX_NUM_CCs][4];
int32_t         uplink_frequency_offset[MAX_NUM_CCs][4];
uint64_t        sidelink_frequency[MAX_NUM_CCs][4];
int             rx_input_level_dBm;

#if MAX_NUM_CCs == 1
rx_gain_t                rx_gain_mode[MAX_NUM_CCs][4] = {{max_gain,max_gain,max_gain,max_gain}};
double tx_gain[MAX_NUM_CCs][4] = {{20,0,0,0}};
double rx_gain[MAX_NUM_CCs][4] = {{110,0,0,0}};
#else
rx_gain_t                rx_gain_mode[MAX_NUM_CCs][4] = {{max_gain,max_gain,max_gain,max_gain},{max_gain,max_gain,max_gain,max_gain}};
double tx_gain[MAX_NUM_CCs][4] = {{20,0,0,0},{20,0,0,0}};
double rx_gain[MAX_NUM_CCs][4] = {{110,0,0,0},{20,0,0,0}};
#endif

// UE and OAI config variables

openair0_config_t openair0_cfg[MAX_CARDS];
int               otg_enabled;
double            cpuf;


int          chain_offset = 0;
int           card_offset = 0;
uint64_t num_missed_slots = 0; // counter for the number of missed slots
int     transmission_mode = 1;
int            numerology = 0;
int           oaisim_flag = 0;
int            emulate_rf = 0;
uint32_t       N_RB_DL    = 106;
uint16_t           Nid_SL = 0;
uint64_t SSB_positions = 0x01;
int mu = 1;
uint8_t n_tx = 1;
uint8_t n_rx = 1;
uint16_t Nid_cell = 0;
int ssb_subcarrier_offset = 0;

/* see file openair2/LAYER2/MAC/main.c for why abstraction_flag is needed
 * this is very hackish - find a proper solution
 */
uint8_t abstraction_flag=0;

nr_bler_struct nr_bler_data[NR_NUM_MCS];
nr_bler_struct nr_mimo_bler_data[NR_NUM_MCS];

static void init_bler_table(void);
static void init_mimo_bler_table(void);

/*---------------------BMC: timespec helpers -----------------------------*/

struct timespec min_diff_time = { .tv_sec = 0, .tv_nsec = 0 };
struct timespec max_diff_time = { .tv_sec = 0, .tv_nsec = 0 };

struct timespec clock_difftime(struct timespec start, struct timespec end) {
  struct timespec temp;

  if ((end.tv_nsec-start.tv_nsec)<0) {
    temp.tv_sec = end.tv_sec-start.tv_sec-1;
    temp.tv_nsec = 1000000000+end.tv_nsec-start.tv_nsec;
  } else {
    temp.tv_sec = end.tv_sec-start.tv_sec;
    temp.tv_nsec = end.tv_nsec-start.tv_nsec;
  }

  return temp;
}

void print_difftimes(void) {
  LOG_I(HW,"difftimes min = %lu ns ; max = %lu ns\n", min_diff_time.tv_nsec, max_diff_time.tv_nsec);
}

int create_tasks_nrue(uint32_t ue_nb) {
  LOG_D(NR_RRC, "%s(ue_nb:%d)\n", __FUNCTION__, ue_nb);
  itti_wait_ready(1);

  if (ue_nb > 0) {
    LOG_I(NR_RRC,"create TASK_RRC_NRUE \n");
    if (itti_create_task (TASK_RRC_NRUE, rrc_nrue_task, NULL) < 0) {
      LOG_E(NR_RRC, "Create task for RRC UE failed\n");
      return -1;
    }
    if (get_softmodem_params()->nsa) {
      init_connections_with_lte_ue();
      if (itti_create_task (TASK_RRC_NSA_NRUE, recv_msgs_from_lte_ue, NULL) < 0) {
        LOG_E(NR_RRC, "Create task for RRC NSA nr-UE failed\n");
        return -1;
      }
    }
    if (itti_create_task (TASK_NAS_NRUE, nas_nrue_task, NULL) < 0) {
      LOG_E(NR_RRC, "Create task for NAS UE failed\n");
      return -1;
    }
  }

  itti_wait_ready(0);
  return 0;
}

void exit_function(const char *file, const char *function, const int line, const char *s) {
  int CC_id;

  if (s != NULL) {
    printf("%s:%d %s() Exiting OAI softmodem: %s\n",file,line, function, s);
  }

  oai_exit = 1;

  if (PHY_vars_UE_g && PHY_vars_UE_g[0]) {
    for(CC_id=0; CC_id<MAX_NUM_CCs; CC_id++) {
      if (PHY_vars_UE_g[0][CC_id] && PHY_vars_UE_g[0][CC_id]->rfdevice.trx_end_func)
        PHY_vars_UE_g[0][CC_id]->rfdevice.trx_end_func(&PHY_vars_UE_g[0][CC_id]->rfdevice);
    }
  }

  sleep(1); //allow lte-softmodem threads to exit first
  exit(1);
}

uint64_t get_nrUE_optmask(void) {
  return nrUE_params.optmask;
}

uint64_t set_nrUE_optmask(uint64_t bitmask) {
  nrUE_params.optmask = nrUE_params.optmask | bitmask;
  return nrUE_params.optmask;
}

nrUE_params_t *get_nrUE_params(void) {
  return &nrUE_params;
}

static void nr_phy_config_request_sl(PHY_VARS_NR_UE *ue,
                                        int N_RB_DL,
                                        int N_RB_UL,
                                        int mu,
                                        int Nid_SL,
                                        int CC_id,
                                        uint64_t position_in_burst)
{
  uint64_t rev_burst = 0;
  for (int i = 0; i < 64; i++)
    rev_burst |= (((position_in_burst >> (63-i))&0x01) << i);

  NR_DL_FRAME_PARMS *fp                                  = &ue->frame_parms;
  fapi_nr_config_request_t *nrUE_config                  = &ue->nrUE_config;
  nrUE_config->cell_config.phy_cell_id                   = Nid_SL; // TODO
  nrUE_config->ssb_config.scs_common                     = mu;
  nrUE_config->ssb_table.ssb_subcarrier_offset           = 0;
  nrUE_config->ssb_table.ssb_offset_point_a              = (fp->N_RB_SL - 11) >> 1;
  nrUE_config->ssb_table.ssb_mask_list[1].ssb_mask       = (rev_burst)&(0xFFFFFFFF);
  nrUE_config->ssb_table.ssb_mask_list[0].ssb_mask       = (rev_burst>>32)&(0xFFFFFFFF);
  nrUE_config->cell_config.frame_duplex_type             = TDD;
  nrUE_config->ssb_table.ssb_period                      = 1; //10ms
  nrUE_config->carrier_config.dl_grid_size[mu]           = N_RB_DL;
  nrUE_config->carrier_config.ul_grid_size[mu]           = N_RB_UL;
  nrUE_config->carrier_config.sl_grid_size[mu]           = fp->N_RB_SL;
  nrUE_config->carrier_config.num_tx_ant                 = fp->nb_antennas_tx;
  nrUE_config->carrier_config.num_rx_ant                 = fp->nb_antennas_rx;
  nrUE_config->tdd_table.tdd_period                      = 0;
  nrUE_config->carrier_config.dl_frequency               = downlink_frequency[CC_id][0] / 1000;
  nrUE_config->carrier_config.uplink_frequency           = downlink_frequency[CC_id][0] / 1000;
  nrUE_config->carrier_config.sl_frequency               = sidelink_frequency[CC_id][0] / 1000;
  LOG_D(NR_PHY, "SL Frequency %u\n", nrUE_config->carrier_config.sl_frequency);
  ue->mac_enabled                                        = 1;
  fp->Ncp                                                = NORMAL;
  fp->tdd_period                                         = 6; // 6 indicates 5ms (see get_nb_periods_per_frame())
  fp->tdd_slot_config                                    = 0b0000111111; // 1 -> UL, 0-> DL for each slot , LSB is the slot 0
  fp->nb_antennas_tx = n_tx;
  fp->nb_antennas_rx = n_rx;
  fp->nb_antenna_ports_gNB = n_tx;
  fp->N_RB_DL = N_RB_DL;
  fp->Nid_cell = Nid_cell;
  fp->Nid_SL = Nid_SL;
  fp->nushift = 0; //No nushift in SL
  fp->ssb_type = nr_ssb_type_C; //Note: case c for NR SL???
  fp->freq_range = mu < 2 ? nr_FR1 : nr_FR2;
  fp->nr_band = get_softmodem_params()->band; //Note: NR SL uses for n38 and n47
  fp->threequarter_fs = 0;
  fp->ofdm_offset_divisor = UINT_MAX;
  fp->first_carrier_offset = 0;
  fp->ssb_start_subcarrier = 12 * ue->nrUE_config.ssb_table.ssb_offset_point_a + ssb_subcarrier_offset;
  nrUE_config->carrier_config.dl_bandwidth = config_bandwidth(mu, N_RB_DL, fp->nr_band);

  ue->slss = calloc(1, sizeof(*ue->slss));
  int len = sizeof(ue->slss->sl_mib) / sizeof(ue->slss->sl_mib[0]);
  for (int i = 0; i < len; i++) {
    ue->slss->sl_mib[i] = 0;
  }
  #if 1
  uint16_t node_id = get_softmodem_params()->node_number;
  ue->slss->sl_mib_length = 32;
  ue->slss->sl_numssb_withinperiod_r16 = 2;
  ue->slss->sl_timeinterval_r16 = 20;
  // TODO: The following setting needs update from MAC layer later.
  ue->slss->sl_timeoffsetssb_r16 = 2;
  if (node_id == 3) {
    ue->slss->sl_timeoffsetssb_r16 = 4;
  }
  ue->slss->sl_numssb_withinperiod_r16_copy = ue->slss->sl_numssb_withinperiod_r16;
  ue->slss->sl_timeinterval_r16_copy = ue->slss->sl_timeinterval_r16;
  ue->slss->sl_timeoffsetssb_r16_copy = ue->slss->sl_timeoffsetssb_r16;
  #endif
  ue->slss->slss_id = Nid_SL;
  ue->is_synchronized_sl = 0;
  ue->UE_fo_compensation = 0;
  ue->UE_scan_carrier = 1;
  ue->sync_ref = get_softmodem_params()->sync_ref;
  ue->configured = true;
  LOG_I(NR_PHY, "nrUE configured\n");
}


static void get_options(void) {

  paramdef_t cmdline_params[] =CMDLINE_NRUEPARAMS_DESC ;
  int numparams = sizeof(cmdline_params)/sizeof(paramdef_t);
  config_get(cmdline_params,numparams,NULL);
  config_process_cmdline( cmdline_params,numparams,NULL);

  if (vcdflag > 0)
    ouput_vcd = 1;
}

// set PHY vars from command line
void set_options(int CC_id, PHY_VARS_NR_UE *UE){
  NR_DL_FRAME_PARMS *fp       = &UE->frame_parms;
  paramdef_t cmdline_params[] = CMDLINE_NRUE_PHYPARAMS_DESC ;
  int numparams               = sizeof(cmdline_params)/sizeof(paramdef_t);

  UE->mode = normal_txrx;

  config_get(cmdline_params,numparams,NULL);

  int pindex = config_paramidx_fromname(cmdline_params,numparams, CALIBRX_OPT);
  if ( (cmdline_params[pindex].paramflags &  PARAMFLAG_PARAMSET) != 0) UE->mode = rx_calib_ue;
  
  pindex = config_paramidx_fromname(cmdline_params,numparams, CALIBRXMED_OPT);
  if ( (cmdline_params[pindex].paramflags &  PARAMFLAG_PARAMSET) != 0) UE->mode = rx_calib_ue_med;

  pindex = config_paramidx_fromname(cmdline_params,numparams, CALIBRXBYP_OPT);              
  if ( (cmdline_params[pindex].paramflags &  PARAMFLAG_PARAMSET) != 0) UE->mode = rx_calib_ue_byp;

  pindex = config_paramidx_fromname(cmdline_params,numparams, DBGPRACH_OPT); 
  if (cmdline_params[pindex].uptr)
    if ( *(cmdline_params[pindex].uptr) > 0) UE->mode = debug_prach;

  pindex = config_paramidx_fromname(cmdline_params,numparams,NOL2CONNECT_OPT ); 
  if (cmdline_params[pindex].uptr)
    if ( *(cmdline_params[pindex].uptr) > 0)  UE->mode = no_L2_connect;

  pindex = config_paramidx_fromname(cmdline_params,numparams,CALIBPRACH_OPT );
  if (cmdline_params[pindex].uptr)
    if ( *(cmdline_params[pindex].uptr) > 0) UE->mode = calib_prach_tx;

  pindex = config_paramidx_fromname(cmdline_params,numparams,DUMPFRAME_OPT );
  if ((cmdline_params[pindex].paramflags & PARAMFLAG_PARAMSET) != 0)
    UE->mode = rx_dump_frame;

  // Init power variables
  tx_max_power[CC_id] = tx_max_power[0];
  rx_gain[0][CC_id]   = rx_gain[0][0];
  tx_gain[0][CC_id]   = tx_gain[0][0];

  // Set UE variables
  UE->rx_total_gain_dB     = (int)rx_gain[CC_id][0] + rx_gain_off;
  UE->tx_total_gain_dB     = (int)tx_gain[CC_id][0];
  UE->tx_power_max_dBm     = tx_max_power[CC_id];
  UE->rf_map.card          = card_offset;
  UE->rf_map.chain         = CC_id + chain_offset;

  LOG_I(PHY,"Set UE mode %d, UE_fo_compensation %d, UE_scan_carrier %d, UE_no_timing_correction %d \n, chest-freq %d\n",
  	   UE->mode, UE->UE_fo_compensation, UE->UE_scan_carrier, UE->no_timing_correction, UE->chest_freq);

  // Set FP variables

  if (tddflag){
    fp->frame_type = TDD;
    LOG_I(PHY, "Set UE frame_type %d\n", fp->frame_type);
  }

  LOG_I(PHY, "Set UE nb_rx_antenna %d, nb_tx_antenna %d, threequarter_fs %d, ssb_start_subcarrier %d\n", fp->nb_antennas_rx, fp->nb_antennas_tx, fp->threequarter_fs, fp->ssb_start_subcarrier);

  fp->ofdm_offset_divisor = nrUE_params.ofdm_offset_divisor;
  UE->max_ldpc_iterations = nrUE_params.max_ldpc_iterations;

}

void init_openair0(void) {
  int card;
  int freq_off = 0;
  NR_DL_FRAME_PARMS *frame_parms = &PHY_vars_UE_g[0][0]->frame_parms;

  for (card=0; card<MAX_CARDS; card++) {
    uint64_t dl_carrier, ul_carrier, sl_carrier;
    openair0_cfg[card].configFilename    = NULL;
    openair0_cfg[card].threequarter_fs   = frame_parms->threequarter_fs;
    openair0_cfg[card].sample_rate       = frame_parms->samples_per_subframe * 1e3;
    openair0_cfg[card].samples_per_frame = frame_parms->samples_per_frame;

    if (frame_parms->frame_type==TDD)
      openair0_cfg[card].duplex_mode = duplex_mode_TDD;
    else
      openair0_cfg[card].duplex_mode = duplex_mode_FDD;

    openair0_cfg[card].Mod_id = 0;
    openair0_cfg[card].num_rb_dl = (get_softmodem_params()->sl_mode == 2) ? frame_parms->N_RB_SL : frame_parms->N_RB_DL;
    openair0_cfg[card].clock_source = get_softmodem_params()->clock_source;
    openair0_cfg[card].time_source = get_softmodem_params()->timing_source;
    openair0_cfg[card].tune_offset = get_softmodem_params()->tune_offset;
    openair0_cfg[card].tx_num_channels = min(4, frame_parms->nb_antennas_tx);
    openair0_cfg[card].rx_num_channels = min(4, frame_parms->nb_antennas_rx);

    LOG_I(PHY, "HW: Configuring card %d, sample_rate %f, tx/rx num_channels %d/%d, duplex_mode %s\n",
      card,
      openair0_cfg[card].sample_rate,
      openair0_cfg[card].tx_num_channels,
      openair0_cfg[card].rx_num_channels,
      duplex_mode[openair0_cfg[card].duplex_mode]);

    nr_get_carrier_frequencies(PHY_vars_UE_g[0][0], &dl_carrier, &ul_carrier);

    nr_rf_card_config_freq(&openair0_cfg[card], ul_carrier, dl_carrier, freq_off);

    if (get_softmodem_params()->sl_mode != 0) {
      nr_get_carrier_frequencies_sl(PHY_vars_UE_g[0][0], &sl_carrier);
      nr_rf_card_config_freq(&openair0_cfg[card], sl_carrier, sl_carrier, freq_off);
    }

    nr_rf_card_config_gain(&openair0_cfg[card], rx_gain_off);

    openair0_cfg[card].configFilename = get_softmodem_params()->rf_config_file;

    if (usrp_args) openair0_cfg[card].sdr_addrs = usrp_args;

  }
}

static void init_pdcp(int ue_id) {
  uint32_t pdcp_initmask = (!IS_SOFTMODEM_NOS1) ? LINK_ENB_PDCP_TO_GTPV1U_BIT : (LINK_ENB_PDCP_TO_GTPV1U_BIT | PDCP_USE_NETLINK_BIT | LINK_ENB_PDCP_TO_IP_DRIVER_BIT);

  /*if (IS_SOFTMODEM_RFSIM || (nfapi_getmode()==NFAPI_UE_STUB_PNF)) {
    pdcp_initmask = pdcp_initmask | UE_NAS_USE_TUN_BIT;
  }*/

  if (IS_SOFTMODEM_NOKRNMOD) {
    pdcp_initmask = pdcp_initmask | UE_NAS_USE_TUN_BIT;
  }
  if (get_softmodem_params()->nsa && rlc_module_init(0) != 0) {
    LOG_I(RLC, "Problem at RLC initiation \n");
  }
  pdcp_layer_init();
  nr_pdcp_module_init(pdcp_initmask, ue_id);
  pdcp_set_rlc_data_req_func((send_rlc_data_req_func_t) rlc_data_req);
  pdcp_set_pdcp_data_ind_func((pdcp_data_ind_func_t) pdcp_data_ind);
}

// Stupid function addition because UE itti messages queues definition is common with eNB
void *rrc_enb_process_msg(void *notUsed) {
  return NULL;
}


int main( int argc, char **argv ) {
  int set_exe_prio = 1;
  if (checkIfFedoraDistribution())
    if (checkIfGenericKernelOnFedora())
      if (checkIfInsideContainer())
        set_exe_prio = 0;
  if (set_exe_prio)
    set_priority(79);

  if (mlockall(MCL_CURRENT | MCL_FUTURE) == -1)
  {
    fprintf(stderr, "mlockall: %s\n", strerror(errno));
    return EXIT_FAILURE;
  }

  //uint8_t beta_ACK=0,beta_RI=0,beta_CQI=2;
  PHY_VARS_NR_UE *UE[MAX_NUM_CCs];
  start_background_system();

  if ( load_configmodule(argc,argv,CONFIG_ENABLECMDLINEONLY) == NULL) {
    exit_fun("[SOFTMODEM] Error, configuration module init failed\n");
  }
  set_softmodem_sighandler();
  CONFIG_SETRTFLAG(CONFIG_NOEXITONHELP);
  memset(openair0_cfg,0,sizeof(openair0_config_t)*MAX_CARDS);
  memset(tx_max_power,0,sizeof(int)*MAX_NUM_CCs);
  // initialize logging
  logInit();
  // get options and fill parameters from configuration file

  get_options (); //Command-line options specific for NRUE

  get_common_options(SOFTMODEM_5GUE_BIT);
  CONFIG_CLEARRTFLAG(CONFIG_NOEXITONHELP);
#if T_TRACER
  T_Config_Init();
#endif
  initTpool(get_softmodem_params()->threadPoolConfig, &(nrUE_params.Tpool), cpumeas(CPUMEAS_GETSTATE));
  //randominit (0);
  set_taus_seed (0);

  cpuf=get_cpu_freq_GHz();
  itti_init(TASK_MAX, tasks_info);

  init_opt() ;
  load_nrLDPClib(NULL);

  if (ouput_vcd) {
    vcd_signal_dumper_init("/tmp/openair_dump_nrUE.vcd");
  }

  #ifndef PACKAGE_VERSION
#  define PACKAGE_VERSION "UNKNOWN-EXPERIMENTAL"
#endif
  LOG_I(HW, "Version: %s\n", PACKAGE_VERSION);

  init_NR_UE(1,uecap_file,rrc_config_path);

  uint16_t node_number = get_softmodem_params()->node_number;
  ue_id_g = (node_number == 0) ? 0 : node_number - 1;
  AssertFatal(ue_id_g >= 0, "UE id is expected to be nonnegative.\n");
  if(IS_SOFTMODEM_NOS1 || get_softmodem_params()->sa || get_softmodem_params()->nsa) {
    if(node_number == 0) {
      init_pdcp(0);
    }
    else {
      init_pdcp(node_number);
    }
  }

  NB_UE_INST=1;
  NB_INST=1;
  PHY_vars_UE_g = malloc(sizeof(PHY_VARS_NR_UE **));
  PHY_vars_UE_g[0] = malloc(sizeof(PHY_VARS_NR_UE *)*MAX_NUM_CCs);
  if (get_softmodem_params()->emulate_l1) {
    RCconfig_nr_ue_macrlc();
    init_bler_table();
    init_mimo_bler_table();
  }

  if (get_softmodem_params()->do_ra)
    AssertFatal(get_softmodem_params()->phy_test == 0,"RA and phy_test are mutually exclusive\n");

  if (get_softmodem_params()->sa)
    AssertFatal(get_softmodem_params()->phy_test == 0,"Standalone mode and phy_test are mutually exclusive\n");

  if (!get_softmodem_params()->nsa && get_softmodem_params()->emulate_l1)
    start_oai_nrue_threads();

  Nid_SL = get_softmodem_params()->nid1 + get_softmodem_params()->nid2 * NUMBER_SSS_SEQUENCE;
  if (!get_softmodem_params()->emulate_l1) {
    for (int CC_id=0; CC_id<MAX_NUM_CCs; CC_id++) {
      PHY_vars_UE_g[0][CC_id] = (PHY_VARS_NR_UE *)malloc(sizeof(PHY_VARS_NR_UE));
      UE[CC_id] = PHY_vars_UE_g[0][CC_id];
      memset(UE[CC_id],0,sizeof(PHY_VARS_NR_UE));
      set_options(CC_id, UE[CC_id]);
      NR_UE_MAC_INST_t *mac = get_mac_inst(0);
      if (get_softmodem_params()->sl_mode == 2) {
        mac->if_module = NULL;
        LOG_I(HW, "Setting mac->if_module = NULL b/c we config PHY in nr_phy_config_request_sl (for now - TODO)\n");
        nr_phy_config_request_sl(UE[CC_id], N_RB_DL, N_RB_DL, mu, Nid_SL, CC_id, SSB_positions);
      }
      if (get_softmodem_params()->sa) {
        uint16_t nr_band = get_band(downlink_frequency[CC_id][0],uplink_frequency_offset[CC_id][0]);
        mac->nr_band = nr_band;
        nr_init_frame_parms_ue_sa(&UE[CC_id]->frame_parms,
                                  downlink_frequency[CC_id][0],
                                  uplink_frequency_offset[CC_id][0],
                                  get_softmodem_params()->numerology,
                                  nr_band);
      } else {
        if(mac->if_module != NULL && mac->if_module->phy_config_request != NULL)
          mac->if_module->phy_config_request(&mac->phy_config);
        nr_init_frame_parms_ue(&UE[CC_id]->frame_parms, &UE[CC_id]->nrUE_config,
                               get_softmodem_params()->sl_mode == 0 ?
                               *mac->scc->downlinkConfigCommon->frequencyInfoDL->frequencyBandList.list.array[0] :
                               UE[CC_id]->frame_parms.nr_band);
      }
      init_nr_ue_vars(UE[CC_id], 0, abstraction_flag);
    }

    init_openair0();
    // init UE_PF_PO and mutex lock
    pthread_mutex_init(&ue_pf_po_mutex, NULL);
    memset (&UE_PF_PO[0][0], 0, sizeof(UE_PF_PO_t)*NUMBER_OF_UE_MAX*MAX_NUM_CCs);
    set_latency_target();
    mlockall(MCL_CURRENT | MCL_FUTURE);
    if (get_softmodem_params()->sl_mode == 2) {
      crcTableInit();
      initTpool("n", &UE[0]->threadPool, true);
      initNotifiedFIFO(&UE[0]->respDecode);
    }
    if(IS_SOFTMODEM_DOSCOPE) {
      load_softscope("nr",PHY_vars_UE_g[0][0]);
    }

    init_NR_UE_threads(1);
    printf("UE threads created by %ld\n", gettid());
  }

  // wait for end of program
  printf("TYPE <CTRL-C> TO TERMINATE\n");

  if (create_tasks_nrue(1) < 0) {
    printf("cannot create ITTI tasks\n");
    exit(-1); // need a softer mode
  }

  // Sleep a while before checking all parameters have been used
  // Some are used directly in external threads, asynchronously
  sleep(20);
  config_check_unknown_cmdlineopt(CONFIG_CHECKALLSECTIONS);

  while(true)
    sleep(3600);

  if (ouput_vcd)
    vcd_signal_dumper_close();

  return 0;
}

// Read in each MCS file and build BLER-SINR-TB table
static void init_bler_table(void) {
  memset(nr_bler_data, 0, sizeof(nr_bler_data));

  const char *awgn_results_dir = getenv("NR_AWGN_RESULTS_DIR");
  if (!awgn_results_dir) {
    LOG_W(NR_MAC, "No $NR_AWGN_RESULTS_DIR\n");
    return;
  }

  for (unsigned int i = 0; i < NR_NUM_MCS; i++) {
    char fName[1024];
    snprintf(fName, sizeof(fName), "%s/mcs%d_awgn_5G.csv", awgn_results_dir, i);
    FILE *pFile = fopen(fName, "r");
    if (!pFile) {
      LOG_E(NR_MAC, "open %s: %s\n", fName, strerror(errno));
      continue;
    }
    size_t bufSize = 1024;
    char * line = NULL;
    char * token;
    char * temp = NULL;
    int nlines = 0;
    while (getline(&line, &bufSize, pFile) > 0) {
      if (!strncmp(line, "SNR", 3)) {
        continue;
      }

      if (nlines > NR_NUM_SINR) {
        LOG_E(NR_MAC, "BLER FILE ERROR - num lines greater than expected - file: %s\n", fName);
        abort();
      }

      token = strtok_r(line, ";", &temp);
      int ncols = 0;
      while (token != NULL) {
        if (ncols > NUM_BLER_COL) {
          LOG_E(NR_MAC, "BLER FILE ERROR - num of cols greater than expected\n");
          abort();
        }

        nr_bler_data[i].bler_table[nlines][ncols] = strtof(token, NULL);
        ncols++;

        token = strtok_r(NULL, ";", &temp);
      }
      nlines++;
    }
    nr_bler_data[i].length = nlines;
    fclose(pFile);
  }
}

// Read in each MCS file and build BLER-SINR-TB table
static void init_mimo_bler_table(void) {
  memset(nr_mimo_bler_data, 0, sizeof(nr_mimo_bler_data));

  const char *awgn_results_dir = getenv("NR_MIMO2x2_AWGN_RESULTS_DIR");
  if (!awgn_results_dir) {
    LOG_W(NR_MAC, "No $NR_MIMO2x2_AWGN_RESULTS_DIR\n");
    return;
  }

  for (unsigned int i = 0; i < NR_NUM_MCS; i++) {
    char fName[1024];
    snprintf(fName, sizeof(fName), "%s/mcs%d_cdlc_mimo2x2_dl.csv", awgn_results_dir, i);
    FILE *pFile = fopen(fName, "r");
    if (!pFile) {
      LOG_E(NR_MAC, "open %s: %s\n", fName, strerror(errno));
      continue;
    }
    size_t bufSize = 1024;
    char * line = NULL;
    char * token;
    char * temp = NULL;
    int nlines = 0;
    while (getline(&line, &bufSize, pFile) > 0) {
      if (!strncmp(line, "SNR", 3)) {
        continue;
      }

      if (nlines > NR_NUM_SINR) {
        LOG_E(NR_MAC, "BLER FILE ERROR - num lines greater than expected - file: %s\n", fName);
        abort();
      }

      token = strtok_r(line, ";", &temp);
      int ncols = 0;
      while (token != NULL) {
        if (ncols > NUM_BLER_COL) {
          LOG_E(NR_MAC, "BLER FILE ERROR - num of cols greater than expected\n");
          abort();
        }

        nr_mimo_bler_data[i].bler_table[nlines][ncols] = strtof(token, NULL);
        ncols++;

        token = strtok_r(NULL, ";", &temp);
      }
      nlines++;
    }
    nr_mimo_bler_data[i].length = nlines;
    fclose(pFile);
  }
}
