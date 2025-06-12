#include "DDR4.h"
#include "DRAM.h"

#include <vector>
#include <functional>
#include <cassert>

using namespace std;
using namespace ramulator;

string DDR4::standard_name = "DDR4";
string DDR4::level_str [int(Level::MAX)] = {"Ch", "Ra", "Bg", "Ba", "Ro", "Co"};

map<string, enum DDR4::Org> DDR4::org_map = {
    {"DDR4_2Gb_x4", DDR4::Org::DDR4_2Gb_x4}, {"DDR4_2Gb_x8", DDR4::Org::DDR4_2Gb_x8}, {"DDR4_2Gb_x16", DDR4::Org::DDR4_2Gb_x16},
    {"DDR4_4Gb_x4", DDR4::Org::DDR4_4Gb_x4}, {"DDR4_4Gb_x8", DDR4::Org::DDR4_4Gb_x8}, {"DDR4_4Gb_x16", DDR4::Org::DDR4_4Gb_x16},
    {"DDR4_8Gb_x4", DDR4::Org::DDR4_8Gb_x4}, {"DDR4_8Gb_x8", DDR4::Org::DDR4_8Gb_x8}, {"DDR4_8Gb_x16", DDR4::Org::DDR4_8Gb_x16},
};

map<string, enum DDR4::Speed> DDR4::speed_map = {
    {"DDR4_1600K", DDR4::Speed::DDR4_1600K}, {"DDR4_1600L", DDR4::Speed::DDR4_1600L},
    {"DDR4_1866M", DDR4::Speed::DDR4_1866M}, {"DDR4_1866N", DDR4::Speed::DDR4_1866N},
    {"DDR4_2133P", DDR4::Speed::DDR4_2133P}, {"DDR4_2133R", DDR4::Speed::DDR4_2133R},
    {"DDR4_2400R", DDR4::Speed::DDR4_2400R}, {"DDR4_2400U", DDR4::Speed::DDR4_2400U},
    {"DDR4_4000", DDR4::Speed::DDR4_4000} // Added 4000MHz
};


DDR4::DDR4(Org org, Speed speed)
    : org_entry(org_table[int(org)]),
      speed_entry(speed_table[int(speed)]), 
      read_latency(speed_entry.nCL + speed_entry.nBL)
{
    init_speed();
    init_prereq();
    init_rowhit(); // SAUGATA: added row hit function
    init_rowopen();
    init_lambda();
    init_timing();
}

DDR4::DDR4(const string& org_str, const string& speed_str) :
    DDR4(org_map[org_str], speed_map[speed_str]) 
{
}

void DDR4::set_channel_number(int channel) {
  org_entry.count[int(Level::Channel)] = channel;
}

void DDR4::set_rank_number(int rank) {
  org_entry.count[int(Level::Rank)] = rank;
}

void DDR4::init_speed()
{
    const static int RRDS_TABLE[2][5] = {
        {4, 4, 4, 4, 4},
        {5, 5, 6, 7, 9}
    };
    const static int RRDL_TABLE[2][5] = {
        {5, 5, 6, 6, 8},
        {6, 6, 7, 8, 11}
    };
    const static int FAW_TABLE[3][5] = {
        {16, 16, 16, 16, 16},
        {20, 22, 23, 26, 34},
        {28, 28, 32, 36, 48}
    };
    const static int RFC_TABLE[int(RefreshMode::MAX)][3][5] = {{   
            {128, 150, 171, 192, 256},
            {208, 243, 278, 312, 416},
            {280, 327, 374, 420, 560}
        },{
            {88, 103, 118, 132,  176},
            {128, 150, 171, 192, 256},
            {208, 243, 278, 312, 416} 
        },{
            {72, 84, 96, 108, 144},
            {88, 103, 118, 132, 176},
            {128, 150, 171, 192, 256}  
        }
    };
    const static int REFI_TABLE[5] = {
        6240, 7280, 8320, 9360, 12480
    };
    const static int XS_TABLE[3][5] = {
        {136, 159, 182, 204, 272},
        {216, 252, 288, 324, 432},
        {288, 336, 384, 432, 576}
    };

    int speed = 0, density = 0;
    switch (speed_entry.rate) {
        case 1600: speed = 0; break;
        case 1866: speed = 1; break;
        case 2133: speed = 2; break;
        case 2400: speed = 3; break;
        case 4000: speed = 4; break;
        default: assert(false);
    };
    switch (org_entry.size >> 10){
        case 2: density = 0; break;
        case 4: density = 1; break;
        case 8: density = 2; break;
        default: assert(false);
    }
    speed_entry.nRRDS = RRDS_TABLE[org_entry.dq == 16? 1: 0][speed];
    speed_entry.nRRDL = RRDL_TABLE[org_entry.dq == 16? 1: 0][speed];
    speed_entry.nFAW = FAW_TABLE[org_entry.dq == 4? 0: org_entry.dq == 8? 1: 2][speed];
    speed_entry.nRFC = RFC_TABLE[(int)refresh_mode][density][speed];
    speed_entry.nREFI = (REFI_TABLE[speed] >> int(refresh_mode));
    speed_entry.nXS = XS_TABLE[density][speed];
}

// Translations from high to lowerlevel commands through prerequisits by current state check
void DDR4::init_prereq()
{
    // RD
    prereq[int(Level::Rank)][int(Command::RD)] = [] (DRAM<DDR4>* node, Command cmd, int id) {
        switch (int(node->state)) {
            case int(State::PowerUp): return Command::MAX;
            case int(State::ActPowerDown): return Command::PDX;
            case int(State::PrePowerDown): return Command::PDX;
            case int(State::SelfRefresh): return Command::SRX;
            default: assert(false);
        }};
    prereq[int(Level::Bank)][int(Command::RD)] = [] (DRAM<DDR4>* node, Command cmd, int id) {
        switch (int(node->state)) {
            case int(State::Closed): return Command::ACT;
            case int(State::Opened):
                if (node->row_state.find(id) != node->row_state.end())
                    return cmd;
                else return Command::PRE;
            default: assert(false);
        }};

    // WR
    prereq[int(Level::Rank)][int(Command::WR)] = prereq[int(Level::Rank)][int(Command::RD)];
    prereq[int(Level::Bank)][int(Command::WR)] = prereq[int(Level::Bank)][int(Command::RD)];

    // REF
    prereq[int(Level::Rank)][int(Command::REF)] = [] (DRAM<DDR4>* node, Command cmd, int id) {
        for (auto bg : node->children)
            for (auto bank: bg->children) {
                if (bank->state == State::Closed)
                    continue;
                return Command::PREA;
            }
        return Command::REF;};

    // PD
    prereq[int(Level::Rank)][int(Command::PDE)] = [] (DRAM<DDR4>* node, Command cmd, int id) {
        switch (int(node->state)) {
            case int(State::PowerUp): return Command::PDE;
            case int(State::ActPowerDown): return Command::PDE;
            case int(State::PrePowerDown): return Command::PDE;
            case int(State::SelfRefresh): return Command::SRX;
            default: assert(false);
        }};

    // SR
    prereq[int(Level::Rank)][int(Command::SRE)] = [] (DRAM<DDR4>* node, Command cmd, int id) {
        switch (int(node->state)) {
            case int(State::PowerUp): return Command::SRE;
            case int(State::ActPowerDown): return Command::PDX;
            case int(State::PrePowerDown): return Command::PDX;
            case int(State::SelfRefresh): return Command::SRE;
            default: assert(false);
        }};
 
    // ROWCLONE:   ACT tRAS -> PREv 3ns -> ACTv tRP-> RC (Back in closed state again)   
    // MAJORITY:   ACT 1.5ns -> PREj 3ns -> ACTv tRP -> MAJ (Back in closed state again)
    // FRACTIONAL: ACT 2.5ns -> PREf tRP -> FRAC (Back in closed state again)

    // RC (Rowclone) | ACT -> PREv -> ACTv
    // TODO: Case in which some Bank state is already open, and need to PRE first (Maybe in scheduler or controller)
    prereq[int(Level::Bank)][int(Command::RC)] = [] (DRAM<DDR4>* node, Command cmd, int id) -> DDR4::Command {
    switch (int(node->state)) {
        case int(State::Closed): return DDR4::Command::ACT;
        case int(State::Opened): return DDR4::Command::PREv;
        case int(State::RowcloneState): return DDR4::Command::ACTv;
        case int(State::ProcessingEnd): return DDR4::Command::RC;
        default: assert(false);
    }};

    // MAJ | ACT -> PREj -> Actv
    // TODO: Case in which some Bank state is already open, and need to PRE first (Maybe in scheduler or controller)
    prereq[int(Level::Bank)][int(Command::MAJ)] = [] (DRAM<DDR4>* node, Command cmd, int id) -> DDR4::Command {
        switch (int(node->state)) {
            case int(State::Closed): return DDR4::Command::ACT;
            case int(State::Opened): return DDR4::Command::PREj;
            case int(State::MajState): return DDR4::Command::ACTv;
            case int(State::ProcessingEnd): return DDR4::Command::MAJ;
            default: assert(false);
    }};

    //            2.5ns     backoff 6 cycles in 2.5ns (7 cycles in total)
    // FRAC | ACT ---> PREf --->
    // Do Frac first for twice for two cells then
    // Always do this twice before a MAJ command, the scheduling will be done in the controller    
    prereq[int(Level::Bank)][int(Command::FRAC)] = [] (DRAM<DDR4>* node, Command cmd, int id) -> DDR4::Command {
        switch (int(node->state)) {
            case int(State::Closed): return DDR4::Command::ACT;
            case int(State::Opened): return DDR4::Command::PREf;
            case int(State::FracState): return DDR4::Command::FRAC; // Change the timing on this
            default: assert(false);
    }};

    // Use the same as Read and Write commands on Rank level to get to normal operation mode energy states
    prereq[int(Level::Rank)][int(Command::RC)] = prereq[int(Level::Rank)][int(Command::RD)];
    prereq[int(Level::Rank)][int(Command::MAJ)] = prereq[int(Level::Rank)][int(Command::RD)];
    prereq[int(Level::Rank)][int(Command::FRAC)] = prereq[int(Level::Rank)][int(Command::RD)];
}

// SAUGATA: added row hit check functions to see if the desired location is currently open
void DDR4::init_rowhit()
{
    // RD
    rowhit[int(Level::Bank)][int(Command::RD)] = [] (DRAM<DDR4>* node, Command cmd, int id) {
        switch (int(node->state)) {
            case int(State::Closed): return false;
            case int(State::Opened):
                if (node->row_state.find(id) != node->row_state.end())
                    return true;
                return false;
            default: assert(false);
        }};

    // WR
    rowhit[int(Level::Bank)][int(Command::WR)] = rowhit[int(Level::Bank)][int(Command::RD)];
}

void DDR4::init_rowopen()
{
    // Maybe reuse this to check if some row is open before issue PuM and then already schedule PRE
    // Check where and how this is used though 
    // RD
    rowopen[int(Level::Bank)][int(Command::RD)] = [] (DRAM<DDR4>* node, Command cmd, int id) {
        switch (int(node->state)) {
            case int(State::Closed): return false;
            case int(State::Opened): return true;
            default: assert(false);
        }};

    // WR
    rowopen[int(Level::Bank)][int(Command::WR)] = rowopen[int(Level::Bank)][int(Command::RD)];
}

// State change mappings
void DDR4::init_lambda()
{
    lambda[int(Level::Bank)][int(Command::ACT)] = [] (DRAM<DDR4>* node, int id) {
        node->state = State::Opened;
        node->row_state[id] = State::Opened;};
    lambda[int(Level::Bank)][int(Command::PRE)] = [] (DRAM<DDR4>* node, int id) {
        node->state = State::Closed;
        node->row_state.clear();};
    lambda[int(Level::Rank)][int(Command::PREA)] = [] (DRAM<DDR4>* node, int id) {
        for (auto bg : node->children)
            for (auto bank : bg->children) {
                bank->state = State::Closed;
                bank->row_state.clear();
            }};
    lambda[int(Level::Rank)][int(Command::REF)] = [] (DRAM<DDR4>* node, int id) {};
    lambda[int(Level::Bank)][int(Command::RD)] = [] (DRAM<DDR4>* node, int id) {};
    lambda[int(Level::Bank)][int(Command::WR)] = [] (DRAM<DDR4>* node, int id) {};
    lambda[int(Level::Bank)][int(Command::RDA)] = [] (DRAM<DDR4>* node, int id) {
        node->state = State::Closed;
        node->row_state.clear();};
    lambda[int(Level::Bank)][int(Command::WRA)] = [] (DRAM<DDR4>* node, int id) {
        node->state = State::Closed;
        node->row_state.clear();};
    lambda[int(Level::Rank)][int(Command::PDE)] = [] (DRAM<DDR4>* node, int id) {
        for (auto bg : node->children)
            for (auto bank : bg->children) {
                if (bank->state == State::Closed)
                    continue;
                node->state = State::ActPowerDown;
                return;
            }
        node->state = State::PrePowerDown;};
    lambda[int(Level::Rank)][int(Command::PDX)] = [] (DRAM<DDR4>* node, int id) {
        node->state = State::PowerUp;};
    lambda[int(Level::Rank)][int(Command::SRE)] = [] (DRAM<DDR4>* node, int id) {
        node->state = State::SelfRefresh;};
    lambda[int(Level::Rank)][int(Command::SRX)] = [] (DRAM<DDR4>* node, int id) {
        node->state = State::PowerUp;};
    
    // State transitions for PuM
    // getting into Rowclone state with PREv
    lambda[int(Level::Bank)][int(Command::PREv)] = [] (DRAM<DDR4>* node, int id) {
        node->state = State::RowcloneState;};
    // Getting into MAJ state with PREj
    lambda[int(Level::Bank)][int(Command::PREj)] = [] (DRAM<DDR4>* node, int id) {
        node->state = State::MajState;};
    // Getting into FRAC state with PREf
    lambda[int(Level::Bank)][int(Command::PREf)] = [] (DRAM<DDR4>* node, int id) {
        node->state = State::FracState;};
    // Getting into the finished processing state after the 32 rows have been opened
    lambda[int(Level::Bank)][int(Command::ACTv)] = [] (DRAM<DDR4>* node, int id) {
        node->state = State::ProcessingEnd;};

    // Closing the states and returning the issued command of PuM
    lambda[int(DDR4::Level::Bank)][int(DDR4::Command::RC)] = [] (DRAM<DDR4>* node, int id) {
        node->state = DDR4::State::Closed;
        node->row_state.clear();};
    lambda[int(DDR4::Level::Bank)][int(DDR4::Command::MAJ)] = [] (DRAM<DDR4>* node, int id) {
        node->state = DDR4::State::Closed;
        node->row_state.clear();};
    lambda[int(DDR4::Level::Bank)][int(DDR4::Command::FRAC)] = [] (DRAM<DDR4>* node, int id) {
        node->state = DDR4::State::Closed;
        node->row_state.clear();};
}

// Timing parameters between commands that are issued
// Watch out for all timings are considered when issuing delays, not just immediate related commands
// For example: (ACT -> FRAC 200ns) (ACTv -> 1ns) (ACT -> ACTv 1ns)
// IF you issued with this ACT -> ACTv -> FRAC
// The it will take 1ns -> 1ns -> 199ns finishing at 201 ns
// Since FRAC needs to be 200ns away from FRAC and 1ns or more away from ACTv 
void DDR4::init_timing()
{
    SpeedEntry& s = speed_entry;
    vector<TimingEntry> *t;

    /*** Channel ***/ 
    t = timing[int(Level::Channel)];

    // CAS <-> CAS
    t[int(Command::RD)].push_back({Command::RD, 1, s.nBL});
    t[int(Command::RD)].push_back({Command::RDA, 1, s.nBL});
    t[int(Command::RDA)].push_back({Command::RD, 1, s.nBL});
    t[int(Command::RDA)].push_back({Command::RDA, 1, s.nBL});
    t[int(Command::WR)].push_back({Command::WR, 1, s.nBL});
    t[int(Command::WR)].push_back({Command::WRA, 1, s.nBL});
    t[int(Command::WRA)].push_back({Command::WR, 1, s.nBL});
    t[int(Command::WRA)].push_back({Command::WRA, 1, s.nBL});
    // PuM does not need to back off if it doesnt need burst lenght (i.e. doesnt send anything back)
    //t[int(Command::MAJ)].push_back({Command::MAJ, 1, 0}); // Time between MAJ commands issued
    //t[int(Command::RC)].push_back({Command::RC, 1, 0}); // time between rowclone commands issued
    //t[int(Command::FRAC)].push_back({Command::FRAC, 1, 0}); // time between Frac commands issued

    /*** Rank ***/ 
    t = timing[int(Level::Rank)];

    // CAS <-> CAS
    t[int(Command::RD)].push_back({Command::RD, 1, s.nCCDS});
    t[int(Command::RD)].push_back({Command::RDA, 1, s.nCCDS});
    t[int(Command::RDA)].push_back({Command::RD, 1, s.nCCDS});
    t[int(Command::RDA)].push_back({Command::RDA, 1, s.nCCDS});
    t[int(Command::WR)].push_back({Command::WR, 1, s.nCCDS});
    t[int(Command::WR)].push_back({Command::WRA, 1, s.nCCDS});
    t[int(Command::WRA)].push_back({Command::WR, 1, s.nCCDS});
    t[int(Command::WRA)].push_back({Command::WRA, 1, s.nCCDS});
    t[int(Command::RD)].push_back({Command::WR, 1, s.nCL + s.nBL + 2 - s.nCWL});
    t[int(Command::RD)].push_back({Command::WRA, 1, s.nCL + s.nBL + 2 - s.nCWL});
    t[int(Command::RDA)].push_back({Command::WR, 1, s.nCL + s.nBL + 2 - s.nCWL});
    t[int(Command::RDA)].push_back({Command::WRA, 1, s.nCL + s.nBL + 2 - s.nCWL});
    t[int(Command::WR)].push_back({Command::RD, 1, s.nCWL + s.nBL + s.nWTRS});
    t[int(Command::WR)].push_back({Command::RDA, 1, s.nCWL + s.nBL + s.nWTRS});
    t[int(Command::WRA)].push_back({Command::RD, 1, s.nCWL + s.nBL + s.nWTRS});
    t[int(Command::WRA)].push_back({Command::RDA, 1, s.nCWL + s.nBL + s.nWTRS});
    
    // Rowclone and MAJ after normal operations
    // rank level parallelism minimum backoff
    t[int(Command::RD)].push_back({Command::RC, 1, s.nCCDS});
    t[int(Command::RD)].push_back({Command::MAJ, 1, s.nCCDS});
    t[int(Command::RDA)].push_back({Command::RC, 1, s.nCCDS});
    t[int(Command::RDA)].push_back({Command::MAJ, 1, s.nCCDS});
    t[int(Command::WR)].push_back({Command::RC, 1, s.nCCDS});
    t[int(Command::WR)].push_back({Command::MAJ, 1, s.nCCDS});
    t[int(Command::WRA)].push_back({Command::RC, 1, s.nCCDS});
    t[int(Command::WRA)].push_back({Command::MAJ, 1, s.nCCDS});

    // Other way around back off
    t[int(Command::RC)].push_back({Command::RD, 1, s.nCCDS});
    t[int(Command::RC)].push_back({Command::RDA, 1, s.nCCDS});
    t[int(Command::MAJ)].push_back({Command::RD, 1, s.nCCDS});
    t[int(Command::MAJ)].push_back({Command::RDA, 1, s.nCCDS});
    t[int(Command::RC)].push_back({Command::WR, 1, s.nCCDS});
    t[int(Command::RC)].push_back({Command::WRA, 1, s.nCCDS});
    t[int(Command::MAJ)].push_back({Command::WR, 1, s.nCCDS});
    t[int(Command::MAJ)].push_back({Command::WRA, 1, s.nCCDS});
    
    // PuM commands to each other
    t[int(Command::RC)].push_back({Command::RC, 1, s.nCCDS});
    t[int(Command::RC)].push_back({Command::MAJ, 1, s.nCCDS});
    t[int(Command::MAJ)].push_back({Command::RC, 1, s.nCCDS});
    t[int(Command::MAJ)].push_back({Command::MAJ, 1, s.nCCDS});


    // Rowclone and MAJ after normal operations
    // rank level parallelism minimum backoff
    t[int(Command::RD)].push_back({Command::FRAC, 1, s.nCCDS});
    t[int(Command::RDA)].push_back({Command::FRAC, 1, s.nCCDS});
    t[int(Command::WR)].push_back({Command::FRAC, 1, s.nCCDS});
    t[int(Command::WRA)].push_back({Command::FRAC, 1, s.nCCDS});

    // Other way around back off
    t[int(Command::FRAC)].push_back({Command::RD, 1, s.nCCDS}); // Maybe set all these to 0, since we dont need backoff from them
    t[int(Command::FRAC)].push_back({Command::RDA, 1, s.nCCDS});
    t[int(Command::FRAC)].push_back({Command::WR, 1, s.nCCDS});
    t[int(Command::FRAC)].push_back({Command::WRA, 1, s.nCCDS});
    
    // PuM commands to each other
    t[int(Command::FRAC)].push_back({Command::RC, 1, s.nCCDS});
    t[int(Command::FRAC)].push_back({Command::MAJ, 1, s.nCCDS});
    t[int(Command::RC)].push_back({Command::FRAC, 1, s.nCCDS});
    t[int(Command::MAJ)].push_back({Command::FRAC, 1, s.nCCDS});
    t[int(Command::FRAC)].push_back({Command::FRAC, 1, s.nCCDS});
    
    // CAS <-> CAS (between sibling ranks)
    t[int(Command::RD)].push_back({Command::RD, 1, s.nBL + s.nRTRS, true});
    t[int(Command::RD)].push_back({Command::RDA, 1, s.nBL + s.nRTRS, true});
    t[int(Command::RDA)].push_back({Command::RD, 1, s.nBL + s.nRTRS, true});
    t[int(Command::RDA)].push_back({Command::RDA, 1, s.nBL + s.nRTRS, true});
    t[int(Command::RD)].push_back({Command::WR, 1, s.nBL + s.nRTRS, true});
    t[int(Command::RD)].push_back({Command::WRA, 1, s.nBL + s.nRTRS, true});
    t[int(Command::RDA)].push_back({Command::WR, 1, s.nBL + s.nRTRS, true});
    t[int(Command::RDA)].push_back({Command::WRA, 1, s.nBL + s.nRTRS, true});
    t[int(Command::WR)].push_back({Command::WR, 1, s.nCL + s.nBL + s.nRTRS - s.nCWL, true});
    t[int(Command::WR)].push_back({Command::WRA, 1, s.nCL + s.nBL + s.nRTRS - s.nCWL, true});
    t[int(Command::WRA)].push_back({Command::WR, 1, s.nCL + s.nBL + s.nRTRS - s.nCWL, true});
    t[int(Command::WRA)].push_back({Command::WRA, 1, s.nCL + s.nBL + s.nRTRS - s.nCWL, true});
    t[int(Command::WR)].push_back({Command::RD, 1, s.nCWL + s.nBL + s.nRTRS - s.nCL, true});
    t[int(Command::WR)].push_back({Command::RDA, 1, s.nCWL + s.nBL + s.nRTRS - s.nCL, true});
    t[int(Command::WRA)].push_back({Command::RD, 1, s.nCWL + s.nBL + s.nRTRS - s.nCL, true});
    t[int(Command::WRA)].push_back({Command::RDA, 1, s.nCWL + s.nBL + s.nRTRS - s.nCL, true});

    //nRTRS (Rank-to-Rank Switching Time)
    //Definition: Minimum delay required when switching between different ranks. for commands that use the data bus (i.e. read/write)
    //Usage: Ensures proper timing when accessing different ranks to prevent data conflicts.
    // Use of rank switching time i.e. nRTRS
    t[int(Command::RD)].push_back({Command::RC, 1, s.nBL + s.nRTRS, true});
    t[int(Command::RD)].push_back({Command::MAJ, 1, s.nBL + s.nRTRS, true});
    t[int(Command::RD)].push_back({Command::FRAC, 1, s.nBL + s.nRTRS, true}); // FRAC does not use the data bus, so not necessary
    t[int(Command::RDA)].push_back({Command::RC, 1, s.nBL + s.nRTRS, true});
    t[int(Command::RDA)].push_back({Command::MAJ, 1, s.nBL + s.nRTRS, true});
    t[int(Command::RDA)].push_back({Command::FRAC, 1, s.nBL + s.nRTRS, true});
    t[int(Command::WR)].push_back({Command::RC, 1, s.nCL + s.nBL + s.nRTRS - s.nCWL, true});
    t[int(Command::WR)].push_back({Command::MAJ, 1, s.nCL + s.nBL + s.nRTRS - s.nCWL, true});
    t[int(Command::WR)].push_back({Command::FRAC, 1, s.nCL + s.nBL + s.nRTRS - s.nCWL, true}); // FRAC does not use the data bus, so not necessary
    t[int(Command::WRA)].push_back({Command::RC, 1, s.nCL + s.nBL + s.nRTRS - s.nCWL, true}); // RC does not use the data bus, so not necessary
    t[int(Command::WRA)].push_back({Command::MAJ, 1, s.nCL + s.nBL + s.nRTRS - s.nCWL, true}); // MAJ does not use the data  bus, so not neceassary
    t[int(Command::WRA)].push_back({Command::FRAC, 1, s.nCL + s.nBL + s.nRTRS - s.nCWL, true}); // FRAC does not use the data bus, so not necessary

    // Other way around back off
    t[int(Command::RC)].push_back({Command::RD, 1, s.nRTRS, true});
    t[int(Command::RC)].push_back({Command::RDA, 1, s.nRTRS, true});
    t[int(Command::MAJ)].push_back({Command::RD, 1, s.nRTRS, true});
    t[int(Command::MAJ)].push_back({Command::RDA, 1,  s.nRTRS, true});
    t[int(Command::FRAC)].push_back({Command::RD, 1, s.nRTRS, true});
    t[int(Command::FRAC)].push_back({Command::RDA, 1, s.nRTRS, true});
    t[int(Command::RC)].push_back({Command::WR, 1,  s.nRTRS, true});
    t[int(Command::RC)].push_back({Command::WRA, 1,  s.nRTRS, true});
    t[int(Command::MAJ)].push_back({Command::WR, 1,  s.nRTRS, true});
    t[int(Command::MAJ)].push_back({Command::WRA, 1,  s.nRTRS, true});
    t[int(Command::FRAC)].push_back({Command::WR, 1,  s.nRTRS, true});
    t[int(Command::FRAC)].push_back({Command::WRA, 1,  s.nRTRS, true});

    // PuM commands to each other
    t[int(Command::RC)].push_back({Command::RC, 1,  s.nRTRS, true});
    t[int(Command::RC)].push_back({Command::MAJ, 1,  s.nRTRS, true});
    t[int(Command::RC)].push_back({Command::FRAC, 1,  s.nRTRS, true});
    t[int(Command::MAJ)].push_back({Command::RC, 1,  s.nRTRS, true});
    t[int(Command::MAJ)].push_back({Command::FRAC, 1,  s.nRTRS, true});
    t[int(Command::MAJ)].push_back({Command::MAJ, 1,  s.nRTRS, true});
    t[int(Command::FRAC)].push_back({Command::RC, 1,  s.nRTRS, true});
    t[int(Command::FRAC)].push_back({Command::MAJ, 1,  s.nRTRS, true});
    t[int(Command::FRAC)].push_back({Command::FRAC, 1,  s.nRTRS, true});

    t[int(Command::RD)].push_back({Command::PREA, 1, s.nRTP});
    t[int(Command::WR)].push_back({Command::PREA, 1, s.nCWL + s.nBL + s.nWR});

    // CAS <-> PD
    t[int(Command::RD)].push_back({Command::PDE, 1, s.nCL + s.nBL + 1});
    t[int(Command::RDA)].push_back({Command::PDE, 1, s.nCL + s.nBL + 1});
    t[int(Command::WR)].push_back({Command::PDE, 1, s.nCWL + s.nBL + s.nWR});
    t[int(Command::WRA)].push_back({Command::PDE, 1, s.nCWL + s.nBL + s.nWR + 1}); // +1 for pre
    t[int(Command::PDX)].push_back({Command::RD, 1, s.nXP});
    t[int(Command::PDX)].push_back({Command::RDA, 1, s.nXP});
    t[int(Command::PDX)].push_back({Command::WR, 1, s.nXP});
    t[int(Command::PDX)].push_back({Command::WRA, 1, s.nXP});

    // PuM to power down timings
    // Onlz s.nCL since no burst length necessary
    t[int(Command::MAJ)].push_back({Command::PDE, 1, s.nCL});
    t[int(Command::RC)].push_back({Command::PDE, 1, s.nCL});
    t[int(Command::FRAC)].push_back({Command::PDE, 1, s.nCL});
    t[int(Command::PDX)].push_back({Command::RC, 1, s.nXP});
    t[int(Command::PDX)].push_back({Command::MAJ, 1, s.nXP});
    t[int(Command::PDX)].push_back({Command::FRAC, 1, s.nXP});
   
    // CAS <-> SR: none (all banks have to be precharged)

    // RAS <-> RAS
    t[int(Command::ACT)].push_back({Command::ACT, 1, s.nRRDS});
    t[int(Command::ACT)].push_back({Command::ACT, 4, s.nFAW}); // Should not interfere but keep in mind that only 4 can be issued quickly
    t[int(Command::ACT)].push_back({Command::PREA, 1, s.nRAS});
    t[int(Command::PREA)].push_back({Command::ACT, 1, s.nRP});

    // RAS <-> REF
    t[int(Command::ACT)].push_back({Command::REF, 1, s.nRC});
    t[int(Command::PRE)].push_back({Command::REF, 1, s.nRP});
    t[int(Command::PREA)].push_back({Command::REF, 1, s.nRP});
    t[int(Command::RDA)].push_back({Command::REF, 1, s.nRTP + s.nRP});
    t[int(Command::WRA)].push_back({Command::REF, 1, s.nCWL + s.nBL + s.nWR + s.nRP});
    // All Pum workloads start with ACT so this also counts for PuM
    t[int(Command::REF)].push_back({Command::ACT, 1, s.nRFC});
    // Since all PuM requests end in PRE, PREv and PREj methods should not be triggered

    // RAS <-> PD
    // Not necessary, no powerdown during or after PuM commands
    t[int(Command::ACT)].push_back({Command::PDE, 1, 1});
    t[int(Command::PDX)].push_back({Command::ACT, 1, s.nXP});
    t[int(Command::PDX)].push_back({Command::PRE, 1, s.nXP});
    t[int(Command::PDX)].push_back({Command::PREA, 1, s.nXP});

    // RAS <-> SR
    t[int(Command::PRE)].push_back({Command::SRE, 1, s.nRP});
    t[int(Command::PREA)].push_back({Command::SRE, 1, s.nRP});
    t[int(Command::SRX)].push_back({Command::ACT, 1, s.nXS});
    // Self refresh should never occur after a PREv or PREj, no refresh during PuM execution

    // REF <-> REF
    t[int(Command::REF)].push_back({Command::REF, 1, s.nRFC});

    // REF <-> PD
    t[int(Command::REF)].push_back({Command::PDE, 1, 1});
    t[int(Command::PDX)].push_back({Command::REF, 1, s.nXP});

    // REF <-> SR
    t[int(Command::SRX)].push_back({Command::REF, 1, s.nXS});
    
    // PD <-> PD
    t[int(Command::PDE)].push_back({Command::PDX, 1, s.nPD});
    t[int(Command::PDX)].push_back({Command::PDE, 1, s.nXP});

    // PD <-> SR
    t[int(Command::PDX)].push_back({Command::SRE, 1, s.nXP});
    t[int(Command::SRX)].push_back({Command::PDE, 1, s.nXS});
    
    // SR <-> SR
    t[int(Command::SRE)].push_back({Command::SRX, 1, s.nCKESR});
    t[int(Command::SRX)].push_back({Command::SRE, 1, s.nXS});

    /*** Bank Group ***/ 
    t = timing[int(Level::BankGroup)];
    // CAS <-> CAS
    t[int(Command::RD)].push_back({Command::RD, 1, s.nCCDL});
    t[int(Command::RD)].push_back({Command::RDA, 1, s.nCCDL});
    t[int(Command::RDA)].push_back({Command::RD, 1, s.nCCDL});
    t[int(Command::RDA)].push_back({Command::RDA, 1, s.nCCDL});
    t[int(Command::WR)].push_back({Command::WR, 1, s.nCCDL});
    t[int(Command::WR)].push_back({Command::WRA, 1, s.nCCDL});
    t[int(Command::WRA)].push_back({Command::WR, 1, s.nCCDL});
    t[int(Command::WRA)].push_back({Command::WRA, 1, s.nCCDL});
    t[int(Command::WR)].push_back({Command::RD, 1, s.nCWL + s.nBL + s.nWTRL});
    t[int(Command::WR)].push_back({Command::RDA, 1, s.nCWL + s.nBL + s.nWTRL});
    t[int(Command::WRA)].push_back({Command::RD, 1, s.nCWL + s.nBL + s.nWTRL});
    t[int(Command::WRA)].push_back({Command::RDA, 1, s.nCWL + s.nBL + s.nWTRL});

    // PuM with itself
    t[int(Command::RC)].push_back({Command::RC, 1, s.nCCDL});
    t[int(Command::MAJ)].push_back({Command::MAJ, 1, s.nCCDL});
    t[int(Command::FRAC)].push_back({Command::FRAC, 1, s.nCCDL});
    t[int(Command::RC)].push_back({Command::MAJ, 1, s.nCCDL});
    t[int(Command::MAJ)].push_back({Command::RC, 1, s.nCCDL});
    t[int(Command::FRAC)].push_back({Command::MAJ, 1, s.nCCDL});
    t[int(Command::FRAC)].push_back({Command::RC, 1, s.nCCDL});
    t[int(Command::RC)].push_back({Command::FRAC, 1, s.nCCDL});
    t[int(Command::MAJ)].push_back({Command::FRAC, 1, s.nCCDL});


    // PuM to others
    t[int(Command::RC)].push_back({Command::RD, 1, s.nCCDL});
    t[int(Command::RC)].push_back({Command::RDA, 1, s.nCCDL});
    t[int(Command::MAJ)].push_back({Command::RD, 1, s.nCCDL});
    t[int(Command::MAJ)].push_back({Command::RDA, 1, s.nCCDL});
    t[int(Command::FRAC)].push_back({Command::RD, 1, s.nCCDL});
    t[int(Command::FRAC)].push_back({Command::RDA, 1, s.nCCDL});
    t[int(Command::RC)].push_back({Command::WR, 1, s.nCCDL});
    t[int(Command::RC)].push_back({Command::WRA, 1, s.nCCDL});
    t[int(Command::MAJ)].push_back({Command::WR, 1, s.nCCDL});
    t[int(Command::MAJ)].push_back({Command::WRA, 1, s.nCCDL});
    t[int(Command::FRAC)].push_back({Command::WR, 1, s.nCCDL});
    t[int(Command::FRAC)].push_back({Command::WRA, 1, s.nCCDL});
    t[int(Command::RD)].push_back({Command::RC, 1, s.nCCDL});
    t[int(Command::RD)].push_back({Command::MAJ, 1, s.nCCDL});
    t[int(Command::RD)].push_back({Command::FRAC, 1, s.nCCDL});
    t[int(Command::RDA)].push_back({Command::RC, 1, s.nCCDL});
    t[int(Command::RDA)].push_back({Command::MAJ, 1, s.nCCDL});
    t[int(Command::RDA)].push_back({Command::FRAC, 1, s.nCCDL});
    t[int(Command::WR)].push_back({Command::RC, 1, s.nCCDL});
    t[int(Command::WR)].push_back({Command::MAJ, 1, s.nCCDL});
    t[int(Command::WR)].push_back({Command::FRAC, 1, s.nCCDL});
    t[int(Command::WRA)].push_back({Command::RC, 1, s.nCCDL});
    t[int(Command::WRA)].push_back({Command::MAJ, 1, s.nCCDL});
    t[int(Command::WRA)].push_back({Command::FRAC, 1, s.nCCDL});

    // RAS <-> RAS
    t[int(Command::ACT)].push_back({Command::ACT, 1, s.nRRDL});
    // All PuM commands start with an ACT so no need for the backoff timing
    // IF we needed here some intra bankgroup backoff, it will limit parallelization
    // Meaning we might need to switch to DDR3 for flat banks
    // But in both ramulator 1 and 2 nRRDL is 0 or -1 so not accounted for anyways so maybe not necessary
    // Only defined in things like HBM etc. but even there it is <= 3 so it doesnt matter since PuM needs 3ns 

    /*** Bank ***/ 
    t = timing[int(Level::Bank)];

    // CAS <-> RAS
    t[int(Command::ACT)].push_back({Command::RD, 1, s.nRCD});
    t[int(Command::ACT)].push_back({Command::RDA, 1, s.nRCD});
    t[int(Command::ACT)].push_back({Command::WR, 1, s.nRCD});
    t[int(Command::ACT)].push_back({Command::WRA, 1, s.nRCD});

    t[int(Command::RD)].push_back({Command::PRE, 1, s.nRTP});
    t[int(Command::WR)].push_back({Command::PRE, 1, s.nCWL + s.nBL + s.nWR});

    t[int(Command::RDA)].push_back({Command::ACT, 1, s.nRTP + s.nRP});
    t[int(Command::WRA)].push_back({Command::ACT, 1, s.nCWL + s.nBL + s.nWR + s.nRP});

    // RAS <-> RAS
    t[int(Command::ACT)].push_back({Command::ACT, 1, s.nRC}); // Causing the issue that PuM wont be issued back to back
    t[int(Command::ACT)].push_back({Command::PRE, 1, s.nRAS});
    t[int(Command::PRE)].push_back({Command::ACT, 1, s.nRP});
    
    // According to the state machine additions 3 new commands are necessary
    // ROWCLONE:   ACT tRAS -> PREv 3ns -> ACTv tRP-> RC (Need to manually issue a PRE after)
    // MAJORITY:   ACT 1.5ns -> PREj 3ns -> ACTv tRP -> MAJ (Back in closed state again)
    // FRACTIONAL: ACT 0.5ns -> PREf tRP -> FRAC (Back in closed state again)

    // Rowclone command timings
    t[int(Command::ACT)].push_back({Command::PREv, 1, s.nRAS});
    t[int(Command::PREv)].push_back({Command::ACTv, 1, 6}); // 3ns
    t[int(Command::ACTv)].push_back({Command::RC, 1, s.nRP}); // No need to additionally wait tRAS since only overwrite

    // MAJ command timings
    t[int(Command::ACT)].push_back({Command::PREj, 1, 3});
    t[int(Command::PREj)].push_back({Command::ACTv, 1, 6});
    t[int(Command::ACTv)].push_back({Command::MAJ, 1, s.nRP}); // They say only wait for tRP, sicne APA is in the tRAS time window

    // FRAC command timings
    t[int(Command::ACT)].push_back({Command::PREf, 1, 1}); // using only one cylce
    t[int(Command::PREf)].push_back({Command::FRAC, 1, s.nRP}); // Wait for the precharge to finish
}
