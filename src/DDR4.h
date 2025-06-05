#ifndef __DDR4_H
#define __DDR4_H

#include "DRAM.h"
#include "Request.h"
#include <vector>
#include <functional>

using namespace std;

namespace ramulator
{

class DDR4
{
public:
    static string standard_name;
    enum class Org;
    enum class Speed;
    DDR4(Org org, Speed speed);
    DDR4(const string& org_str, const string& speed_str);
    
    static map<string, enum Org> org_map;
    static map<string, enum Speed> speed_map;
    /* Level */
    enum class Level : int
    { 
        Channel, Rank, BankGroup, Bank, Row, Column, MAX
    };
    
    static std::string level_str [int(Level::MAX)];

    /* Command */
    enum class Command : int
    { 
        ACT, PRE, PREA, 
        RD,  WR,  RDA,  WRA, 
        REF, PDE, PDX,  SRE, SRX,
        RC, MAJ, FRAC, ACTv, PREv, PREj, PREf, 
        MAX
    };

    string command_name[int(Command::MAX)] = {
        "ACT", "PRE", "PREA", 
        "RD",  "WR",  "RDA",  "WRA", 
        "REF", "PDE", "PDX",  "SRE", "SRX",
        "RC", "MAJ", "FRAC","ACTv", "PREv", "PREj", "PREf", // Register PuM command names and the news violating low level commands
    };

    Level scope[int(Command::MAX)] = {
        Level::Row,    Level::Bank,   Level::Rank,   
        Level::Column, Level::Column, Level::Column, Level::Column,
        Level::Rank,   Level::Rank,   Level::Rank,   Level::Rank,   Level::Rank, 
        Level::Bank, Level::Bank, Level::Bank, Level::Row, Level::Bank, Level::Bank, Level::Bank, // Add the level of the new commands
    };

    bool is_opening(Command cmd) 
    {
        switch(int(cmd)) {
            case int(Command::ACT):
                return true;
            default:
                return false;
        }
    }

    bool is_accessing(Command cmd) 
    {
        switch(int(cmd)) {
            case int(Command::RD):
            case int(Command::WR):
            case int(Command::RDA):
            case int(Command::WRA):
                return true;
            default:
                return false;
        }
    }

    bool is_closing(Command cmd) 
    {
        switch(int(cmd)) {
            case int(Command::RDA):
            case int(Command::WRA):
            case int(Command::PRE):
            case int(Command::PREA):
                return true;
            default:
                return false;
        }
    }

    // From Acting state to rowclone state
    bool is_starting_rowclone(Command cmd)
    {
        switch(int(cmd)) {
            case int(Command::PREv):
                return true;
            default:
                return false;
        }
    }

    // From Active state to maj state with PREj
    bool is_starting_maj(Command cmd)
    {
        switch(int(cmd)) {
            case int(Command::PREj):
                return true;
            default:
                return false;
        }
    }

    // From rowclone state back to idle
    bool is_ending_pum(Command cmd)
    {
        switch (int(cmd)) {
        case int(Command::PRE):
            return true;
        default:
            return false;
        }
    }

    bool is_refreshing(Command cmd) 
    {
        switch(int(cmd)) {
            case int(Command::REF):
                return true;
            default:
                return false;
        }
    }

    /* State */
    enum class State : int
    {
        Opened, Closed, PowerUp, ActPowerDown, PrePowerDown, SelfRefresh, MajState, RowcloneState, ProcessingEnd, FracState, MAX
    } start[int(Level::MAX)] = {
        State::MAX, State::PowerUp, State::MAX, State::Closed, State::Closed, State::MAX
    };

    /* Translate */
    Command translate[int(Request::Type::MAX)] = {
        Command::RD,  Command::WR,
        Command::REF, Command::PDE, Command::SRE,
        Command::RC, Command::MAJ, Command::FRAC,
    };

    /* Prereq */
    function<Command(DRAM<DDR4>*, Command cmd, int)> prereq[int(Level::MAX)][int(Command::MAX)];

    // SAUGATA: added function object container for row hit status
    /* Row hit */
    function<bool(DRAM<DDR4>*, Command cmd, int)> rowhit[int(Level::MAX)][int(Command::MAX)];
    function<bool(DRAM<DDR4>*, Command cmd, int)> rowopen[int(Level::MAX)][int(Command::MAX)];

    /* Timing */
    struct TimingEntry
    {
        Command cmd;
        int dist;
        int val;
        bool sibling;
    }; 
    vector<TimingEntry> timing[int(Level::MAX)][int(Command::MAX)];

    /* Lambda */
    function<void(DRAM<DDR4>*, int)> lambda[int(Level::MAX)][int(Command::MAX)];

    /* Organization */
    enum class Org : int
    {
        DDR4_2Gb_x4,   DDR4_2Gb_x8,   DDR4_2Gb_x16,
        DDR4_4Gb_x4,   DDR4_4Gb_x8,   DDR4_4Gb_x16,
        DDR4_8Gb_x4,   DDR4_8Gb_x8,   DDR4_8Gb_x16,
        MAX
    };

    struct OrgEntry {
        int size;
        int dq;
        int count[int(Level::MAX)];
    } org_table[int(Org::MAX)] = {
        {2<<10,  4, {0, 0, 4, 4, 1<<15, 1<<10}}, {2<<10,  8, {0, 0, 4, 4, 1<<14, 1<<10}}, {2<<10, 16, {0, 0, 2, 4, 1<<14, 1<<10}},
        {4<<10,  4, {0, 0, 4, 4, 1<<16, 1<<10}}, {4<<10,  8, {0, 0, 4, 4, 1<<15, 1<<10}}, {4<<10, 16, {0, 0, 2, 4, 1<<15, 1<<10}},
        {8<<10,  4, {0, 0, 4, 4, 1<<17, 1<<10}}, {8<<10,  8, {0, 0, 4, 4, 1<<16, 1<<10}}, {8<<10, 16, {0, 0, 2, 4, 1<<16, 1<<10}}
    }, org_entry;

    void set_channel_number(int channel);
    void set_rank_number(int rank);

    /* Speed */
    enum class Speed : int
    {
        DDR4_1600K, DDR4_1600L,
        DDR4_1866M, DDR4_1866N,
        DDR4_2133P, DDR4_2133R,
        DDR4_2400R, DDR4_2400U,
        DDR4_3200, DDR4_4000,
        MAX
    };

    enum class RefreshMode : int
    {
        Refresh_1X,
        Refresh_2X,
        Refresh_4X,
        MAX
    } refresh_mode = RefreshMode::Refresh_1X;

    int prefetch_size = 8; // 8n prefetch DDR
    int channel_width = 64;

    struct SpeedEntry {
        // For PuM timing definitions look in the DDR4.cpp
        // PuM timings are alwazs required to be the same, so statically inside the init_timings() of DDR4
        int rate;
        double freq, tCK;
        int nBL, nCCDS, nCCDL, nRTRS;
        int nCL, nRCD, nRP, nCWL;
        int nRAS, nRC;
        int nRTP, nWTRS, nWTRL, nWR;
        int nRRDS, nRRDL, nFAW;
        int nRFC, nREFI;
        int nPD, nXP, nXPDLL; // XPDLL not found in DDR4??
        int nCKESR, nXS, nXSDLL; // nXSDLL TBD (nDLLK), nXS = (tRFC+10ns)/tCK 
    } speed_table[int(Speed::MAX)] = {
        {1600, (400.0/3)*6, (3/0.4)/6, 4, 4, 5, 2, 11, 11, 11,  9, 28, 39, 6, 2, 6, 12, 0, 0, 0, 0, 0, 4, 5, 0, 5, 0, 0},
        {1600, (400.0/3)*6, (3/0.4)/6, 4, 4, 5, 2, 12, 12, 12,  9, 28, 40, 6, 2, 6, 12, 0, 0, 0, 0, 0, 4, 5, 0, 5, 0, 0},
        {1866, (400.0/3)*7, (3/0.4)/7, 4, 4, 5, 2, 13, 13, 13, 10, 32, 45, 7, 3, 7, 14, 0, 0, 0, 0, 0, 5, 6, 0, 6, 0, 0},
        {1866, (400.0/3)*7, (3/0.4)/7, 4, 4, 5, 2, 14, 14, 14, 10, 32, 46, 7, 3, 7, 14, 0, 0, 0, 0, 0, 5, 6, 0, 6, 0, 0},
        {2133, (400.0/3)*8, (3/0.4)/8, 4, 4, 6, 2, 15, 15, 15, 11, 36, 51, 8, 3, 8, 16, 0, 0, 0, 0, 0, 6, 7, 0, 7, 0, 0},
        {2133, (400.0/3)*8, (3/0.4)/8, 4, 4, 6, 2, 16, 16, 16, 11, 36, 52, 8, 3, 8, 16, 0, 0, 0, 0, 0, 6, 7, 0, 7, 0, 0},
        {2400, (400.0/3)*9, (3/0.4)/9, 4, 4, 6, 2, 16, 16, 16, 12, 39, 55, 9, 3, 9, 18, 0, 0, 0, 0, 0, 6, 8, 0, 7, 0, 0},
        {2400, (400.0/3)*9, (3/0.4)/9, 4, 4, 6, 2, 18, 18, 18, 12, 39, 57, 9, 3, 9, 18, 0, 0, 0, 0, 0, 6, 8, 0, 7, 0, 0},
        {3200, 1600, 0.625, prefetch_size/2/*DDR*/, 4,     10,   2,    22, 22,  22, 16,  56,  78, 12,  4,    12,   24, 8,    10,   40,  0,   0,    8,  10, 0,     8,     0,  0},
        {
            4000, 2000, 0.5,    // rate (MT/s), freq (MHz), tCK (ns)
            4,     // tBL     = Burst Length (4)
            4,     // tCCD_S  = CAS-to-CAS Delay (same bank group)
            10,    // tCCD_L  = CAS-to-CAS Delay (diff bank group)
            2,     // tRTRS   = Rank-to-Rank Switching Time
            16,    // tCL     = CAS Latency
            21,    // tRCD    = Row to Column Delay
            18,    // tRP     = Row Precharge Time
            16,    // tCWL    = CAS Write Latency
            40,    // tRAS    = Row Active Time
            62,    // tRC     = Row Cycle Time (tRAS + tRP)
            12,    // tRTP    = Read to Precharge
            5,     // tWTR_S  = Write to Read (same bank group)
            13,    // tWTR_L  = Write to Read (diff bank group)
            24,    // tWR     = Write Recovery
            5,    // tRRD_S  = Activate to Activate (same bank group)
            7,    // tRRD_L  = Activate to Activate (diff bank group)
            20,    // tFAW    = Four Activate Window
            0,     // tRFC    = Refresh Cycle Time (TBD)
            0,     // tREFI   = Refresh Interval (TBD)
            9,     // tPD     = Power-Down Exit
            12,    // tXP     = Exit Power-Down to valid command
            0,     // tXPDLL  = Exit DLL power-down (not always applicable)
            9,     // tCKESR  = CKE Minimum Pulse Width for Self-Refresh
            0,     // tXS     = Exit Self-Refresh (≈ tRFC + 10ns)/tCK
            0      // tXSDLL  = Exit Self-Refresh DLL Lock Time
        },
    }, speed_entry;

    int read_latency;
    //int rowclone_latency;
    //int maj_latency;
    //int frac_latency;

private:
    void init_speed();
    void init_lambda();
    void init_prereq();
    void init_rowhit();  // SAUGATA: added function to check for row hits
    void init_rowopen();
    void init_timing();
};

} /*namespace ramulator*/

#endif /*__DDR4_H*/
