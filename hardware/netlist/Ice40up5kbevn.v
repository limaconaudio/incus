// Generator : SpinalHDL v1.3.6    git head : 4dddb981b1ff8b69e5a0ebe13b502868a1bcff0d
// Date      : 06/07/2019, 22:10:32
// Component : Ice40up5kbevn


`define UartStopType_defaultEncoding_type [0:0]
`define UartStopType_defaultEncoding_ONE 1'b0
`define UartStopType_defaultEncoding_TWO 1'b1

`define UartParityType_defaultEncoding_type [1:0]
`define UartParityType_defaultEncoding_NONE 2'b00
`define UartParityType_defaultEncoding_EVEN 2'b01
`define UartParityType_defaultEncoding_ODD 2'b10

`define UartCtrlTxState_defaultEncoding_type [2:0]
`define UartCtrlTxState_defaultEncoding_IDLE 3'b000
`define UartCtrlTxState_defaultEncoding_START 3'b001
`define UartCtrlTxState_defaultEncoding_DATA 3'b010
`define UartCtrlTxState_defaultEncoding_PARITY 3'b011
`define UartCtrlTxState_defaultEncoding_STOP 3'b100

`define UartCtrlRxState_defaultEncoding_type [2:0]
`define UartCtrlRxState_defaultEncoding_IDLE 3'b000
`define UartCtrlRxState_defaultEncoding_START 3'b001
`define UartCtrlRxState_defaultEncoding_DATA 3'b010
`define UartCtrlRxState_defaultEncoding_PARITY 3'b011
`define UartCtrlRxState_defaultEncoding_STOP 3'b100

`define AluBitwiseCtrlEnum_defaultEncoding_type [1:0]
`define AluBitwiseCtrlEnum_defaultEncoding_XOR_1 2'b00
`define AluBitwiseCtrlEnum_defaultEncoding_OR_1 2'b01
`define AluBitwiseCtrlEnum_defaultEncoding_AND_1 2'b10

`define EnvCtrlEnum_defaultEncoding_type [1:0]
`define EnvCtrlEnum_defaultEncoding_NONE 2'b00
`define EnvCtrlEnum_defaultEncoding_XRET 2'b01
`define EnvCtrlEnum_defaultEncoding_ECALL 2'b10

`define ShiftCtrlEnum_defaultEncoding_type [1:0]
`define ShiftCtrlEnum_defaultEncoding_DISABLE_1 2'b00
`define ShiftCtrlEnum_defaultEncoding_SLL_1 2'b01
`define ShiftCtrlEnum_defaultEncoding_SRL_1 2'b10
`define ShiftCtrlEnum_defaultEncoding_SRA_1 2'b11

`define AluCtrlEnum_defaultEncoding_type [1:0]
`define AluCtrlEnum_defaultEncoding_ADD_SUB 2'b00
`define AluCtrlEnum_defaultEncoding_SLT_SLTU 2'b01
`define AluCtrlEnum_defaultEncoding_BITWISE 2'b10

`define BranchCtrlEnum_defaultEncoding_type [1:0]
`define BranchCtrlEnum_defaultEncoding_INC 2'b00
`define BranchCtrlEnum_defaultEncoding_B 2'b01
`define BranchCtrlEnum_defaultEncoding_JAL 2'b10
`define BranchCtrlEnum_defaultEncoding_JALR 2'b11

`define Src2CtrlEnum_defaultEncoding_type [1:0]
`define Src2CtrlEnum_defaultEncoding_RS 2'b00
`define Src2CtrlEnum_defaultEncoding_IMI 2'b01
`define Src2CtrlEnum_defaultEncoding_IMS 2'b10
`define Src2CtrlEnum_defaultEncoding_PC 2'b11

`define Src1CtrlEnum_defaultEncoding_type [1:0]
`define Src1CtrlEnum_defaultEncoding_RS 2'b00
`define Src1CtrlEnum_defaultEncoding_IMU 2'b01
`define Src1CtrlEnum_defaultEncoding_PC_INCREMENT 2'b10
`define Src1CtrlEnum_defaultEncoding_URS1 2'b11

`define JtagState_defaultEncoding_type [3:0]
`define JtagState_defaultEncoding_RESET 4'b0000
`define JtagState_defaultEncoding_IDLE 4'b0001
`define JtagState_defaultEncoding_IR_SELECT 4'b0010
`define JtagState_defaultEncoding_IR_CAPTURE 4'b0011
`define JtagState_defaultEncoding_IR_SHIFT 4'b0100
`define JtagState_defaultEncoding_IR_EXIT1 4'b0101
`define JtagState_defaultEncoding_IR_PAUSE 4'b0110
`define JtagState_defaultEncoding_IR_EXIT2 4'b0111
`define JtagState_defaultEncoding_IR_UPDATE 4'b1000
`define JtagState_defaultEncoding_DR_SELECT 4'b1001
`define JtagState_defaultEncoding_DR_CAPTURE 4'b1010
`define JtagState_defaultEncoding_DR_SHIFT 4'b1011
`define JtagState_defaultEncoding_DR_EXIT1 4'b1100
`define JtagState_defaultEncoding_DR_PAUSE 4'b1101
`define JtagState_defaultEncoding_DR_EXIT2 4'b1110
`define JtagState_defaultEncoding_DR_UPDATE 4'b1111

`define mapping_xip_fsm_enumDefinition_defaultEncoding_type [2:0]
`define mapping_xip_fsm_enumDefinition_defaultEncoding_boot 3'b000
`define mapping_xip_fsm_enumDefinition_defaultEncoding_mapping_xip_fsm_IDLE 3'b001
`define mapping_xip_fsm_enumDefinition_defaultEncoding_mapping_xip_fsm_INSTRUCTION 3'b010
`define mapping_xip_fsm_enumDefinition_defaultEncoding_mapping_xip_fsm_ADDRESS 3'b011
`define mapping_xip_fsm_enumDefinition_defaultEncoding_mapping_xip_fsm_DUMMY 3'b100
`define mapping_xip_fsm_enumDefinition_defaultEncoding_mapping_xip_fsm_PAYLOAD 3'b101
`define mapping_xip_fsm_enumDefinition_defaultEncoding_mapping_xip_fsm_STOP 3'b110

module BufferCC (
      input   io_initial,
      input   io_dataIn,
      output  io_dataOut,
      input   clk_12M,
      input   clockCtrl_systemReset);
  reg  buffers_0;
  reg  buffers_1;
  assign io_dataOut = buffers_1;
  always @ (posedge clk_12M) begin
    if(clockCtrl_systemReset) begin
      buffers_0 <= io_initial;
      buffers_1 <= io_initial;
    end else begin
      buffers_0 <= io_dataIn;
      buffers_1 <= buffers_0;
    end
  end

endmodule

module UartCtrlTx (
      input  [2:0] io_configFrame_dataLength,
      input  `UartStopType_defaultEncoding_type io_configFrame_stop,
      input  `UartParityType_defaultEncoding_type io_configFrame_parity,
      input   io_samplingTick,
      input   io_write_valid,
      output reg  io_write_ready,
      input  [7:0] io_write_payload,
      output  io_txd,
      input   clk_12M,
      input   clockCtrl_systemReset);
  wire  _zz_1_;
  wire [0:0] _zz_2_;
  wire [2:0] _zz_3_;
  wire [0:0] _zz_4_;
  wire [2:0] _zz_5_;
  reg  clockDivider_counter_willIncrement;
  wire  clockDivider_counter_willClear;
  reg [2:0] clockDivider_counter_valueNext;
  reg [2:0] clockDivider_counter_value;
  wire  clockDivider_counter_willOverflowIfInc;
  wire  clockDivider_tick;
  reg [2:0] tickCounter_value;
  reg `UartCtrlTxState_defaultEncoding_type stateMachine_state;
  reg  stateMachine_parity;
  reg  stateMachine_txd;
  reg  stateMachine_txd_regNext;
  `ifndef SYNTHESIS
  reg [23:0] io_configFrame_stop_string;
  reg [31:0] io_configFrame_parity_string;
  reg [47:0] stateMachine_state_string;
  `endif

  assign _zz_1_ = (tickCounter_value == io_configFrame_dataLength);
  assign _zz_2_ = clockDivider_counter_willIncrement;
  assign _zz_3_ = {2'd0, _zz_2_};
  assign _zz_4_ = ((io_configFrame_stop == `UartStopType_defaultEncoding_ONE) ? (1'b0) : (1'b1));
  assign _zz_5_ = {2'd0, _zz_4_};
  `ifndef SYNTHESIS
  always @(*) begin
    case(io_configFrame_stop)
      `UartStopType_defaultEncoding_ONE : io_configFrame_stop_string = "ONE";
      `UartStopType_defaultEncoding_TWO : io_configFrame_stop_string = "TWO";
      default : io_configFrame_stop_string = "???";
    endcase
  end
  always @(*) begin
    case(io_configFrame_parity)
      `UartParityType_defaultEncoding_NONE : io_configFrame_parity_string = "NONE";
      `UartParityType_defaultEncoding_EVEN : io_configFrame_parity_string = "EVEN";
      `UartParityType_defaultEncoding_ODD : io_configFrame_parity_string = "ODD ";
      default : io_configFrame_parity_string = "????";
    endcase
  end
  always @(*) begin
    case(stateMachine_state)
      `UartCtrlTxState_defaultEncoding_IDLE : stateMachine_state_string = "IDLE  ";
      `UartCtrlTxState_defaultEncoding_START : stateMachine_state_string = "START ";
      `UartCtrlTxState_defaultEncoding_DATA : stateMachine_state_string = "DATA  ";
      `UartCtrlTxState_defaultEncoding_PARITY : stateMachine_state_string = "PARITY";
      `UartCtrlTxState_defaultEncoding_STOP : stateMachine_state_string = "STOP  ";
      default : stateMachine_state_string = "??????";
    endcase
  end
  `endif

  always @ (*) begin
    clockDivider_counter_willIncrement = 1'b0;
    if(io_samplingTick)begin
      clockDivider_counter_willIncrement = 1'b1;
    end
  end

  assign clockDivider_counter_willClear = 1'b0;
  assign clockDivider_counter_willOverflowIfInc = (clockDivider_counter_value == (3'b100));
  assign clockDivider_tick = (clockDivider_counter_willOverflowIfInc && clockDivider_counter_willIncrement);
  always @ (*) begin
    if(clockDivider_tick)begin
      clockDivider_counter_valueNext = (3'b000);
    end else begin
      clockDivider_counter_valueNext = (clockDivider_counter_value + _zz_3_);
    end
    if(clockDivider_counter_willClear)begin
      clockDivider_counter_valueNext = (3'b000);
    end
  end

  always @ (*) begin
    stateMachine_txd = 1'b1;
    case(stateMachine_state)
      `UartCtrlTxState_defaultEncoding_IDLE : begin
      end
      `UartCtrlTxState_defaultEncoding_START : begin
        stateMachine_txd = 1'b0;
      end
      `UartCtrlTxState_defaultEncoding_DATA : begin
        stateMachine_txd = io_write_payload[tickCounter_value];
      end
      `UartCtrlTxState_defaultEncoding_PARITY : begin
        stateMachine_txd = stateMachine_parity;
      end
      default : begin
      end
    endcase
  end

  always @ (*) begin
    io_write_ready = 1'b0;
    case(stateMachine_state)
      `UartCtrlTxState_defaultEncoding_IDLE : begin
      end
      `UartCtrlTxState_defaultEncoding_START : begin
      end
      `UartCtrlTxState_defaultEncoding_DATA : begin
        if(clockDivider_tick)begin
          if(_zz_1_)begin
            io_write_ready = 1'b1;
          end
        end
      end
      `UartCtrlTxState_defaultEncoding_PARITY : begin
      end
      default : begin
      end
    endcase
  end

  assign io_txd = stateMachine_txd_regNext;
  always @ (posedge clk_12M) begin
    if(clockCtrl_systemReset) begin
      clockDivider_counter_value <= (3'b000);
      stateMachine_state <= `UartCtrlTxState_defaultEncoding_IDLE;
      stateMachine_txd_regNext <= 1'b1;
    end else begin
      clockDivider_counter_value <= clockDivider_counter_valueNext;
      case(stateMachine_state)
        `UartCtrlTxState_defaultEncoding_IDLE : begin
          if((io_write_valid && clockDivider_tick))begin
            stateMachine_state <= `UartCtrlTxState_defaultEncoding_START;
          end
        end
        `UartCtrlTxState_defaultEncoding_START : begin
          if(clockDivider_tick)begin
            stateMachine_state <= `UartCtrlTxState_defaultEncoding_DATA;
          end
        end
        `UartCtrlTxState_defaultEncoding_DATA : begin
          if(clockDivider_tick)begin
            if(_zz_1_)begin
              if((io_configFrame_parity == `UartParityType_defaultEncoding_NONE))begin
                stateMachine_state <= `UartCtrlTxState_defaultEncoding_STOP;
              end else begin
                stateMachine_state <= `UartCtrlTxState_defaultEncoding_PARITY;
              end
            end
          end
        end
        `UartCtrlTxState_defaultEncoding_PARITY : begin
          if(clockDivider_tick)begin
            stateMachine_state <= `UartCtrlTxState_defaultEncoding_STOP;
          end
        end
        default : begin
          if(clockDivider_tick)begin
            if((tickCounter_value == _zz_5_))begin
              stateMachine_state <= (io_write_valid ? `UartCtrlTxState_defaultEncoding_START : `UartCtrlTxState_defaultEncoding_IDLE);
            end
          end
        end
      endcase
      stateMachine_txd_regNext <= stateMachine_txd;
    end
  end

  always @ (posedge clk_12M) begin
    if(clockDivider_tick)begin
      tickCounter_value <= (tickCounter_value + (3'b001));
    end
    if(clockDivider_tick)begin
      stateMachine_parity <= (stateMachine_parity ^ stateMachine_txd);
    end
    case(stateMachine_state)
      `UartCtrlTxState_defaultEncoding_IDLE : begin
      end
      `UartCtrlTxState_defaultEncoding_START : begin
        if(clockDivider_tick)begin
          stateMachine_parity <= (io_configFrame_parity == `UartParityType_defaultEncoding_ODD);
          tickCounter_value <= (3'b000);
        end
      end
      `UartCtrlTxState_defaultEncoding_DATA : begin
        if(clockDivider_tick)begin
          if(_zz_1_)begin
            tickCounter_value <= (3'b000);
          end
        end
      end
      `UartCtrlTxState_defaultEncoding_PARITY : begin
        if(clockDivider_tick)begin
          tickCounter_value <= (3'b000);
        end
      end
      default : begin
      end
    endcase
  end

endmodule

module UartCtrlRx (
      input  [2:0] io_configFrame_dataLength,
      input  `UartStopType_defaultEncoding_type io_configFrame_stop,
      input  `UartParityType_defaultEncoding_type io_configFrame_parity,
      input   io_samplingTick,
      output  io_read_valid,
      output [7:0] io_read_payload,
      input   io_rxd,
      input   clk_12M,
      input   clockCtrl_systemReset);
  wire  _zz_1_;
  wire  bufferCC_4__io_dataOut;
  wire  _zz_2_;
  wire  _zz_3_;
  wire  _zz_4_;
  wire [0:0] _zz_5_;
  wire [2:0] _zz_6_;
  wire  sampler_synchroniser;
  wire  sampler_samples_0;
  reg  sampler_samples_1;
  reg  sampler_samples_2;
  reg  sampler_value;
  reg  sampler_tick;
  reg [2:0] bitTimer_counter;
  reg  bitTimer_tick;
  reg [2:0] bitCounter_value;
  reg `UartCtrlRxState_defaultEncoding_type stateMachine_state;
  reg  stateMachine_parity;
  reg [7:0] stateMachine_shifter;
  reg  stateMachine_validReg;
  `ifndef SYNTHESIS
  reg [23:0] io_configFrame_stop_string;
  reg [31:0] io_configFrame_parity_string;
  reg [47:0] stateMachine_state_string;
  `endif

  assign _zz_2_ = (bitTimer_counter == (3'b000));
  assign _zz_3_ = (sampler_tick && (! sampler_value));
  assign _zz_4_ = (bitCounter_value == io_configFrame_dataLength);
  assign _zz_5_ = ((io_configFrame_stop == `UartStopType_defaultEncoding_ONE) ? (1'b0) : (1'b1));
  assign _zz_6_ = {2'd0, _zz_5_};
  BufferCC bufferCC_4_ ( 
    .io_initial(_zz_1_),
    .io_dataIn(io_rxd),
    .io_dataOut(bufferCC_4__io_dataOut),
    .clk_12M(clk_12M),
    .clockCtrl_systemReset(clockCtrl_systemReset) 
  );
  `ifndef SYNTHESIS
  always @(*) begin
    case(io_configFrame_stop)
      `UartStopType_defaultEncoding_ONE : io_configFrame_stop_string = "ONE";
      `UartStopType_defaultEncoding_TWO : io_configFrame_stop_string = "TWO";
      default : io_configFrame_stop_string = "???";
    endcase
  end
  always @(*) begin
    case(io_configFrame_parity)
      `UartParityType_defaultEncoding_NONE : io_configFrame_parity_string = "NONE";
      `UartParityType_defaultEncoding_EVEN : io_configFrame_parity_string = "EVEN";
      `UartParityType_defaultEncoding_ODD : io_configFrame_parity_string = "ODD ";
      default : io_configFrame_parity_string = "????";
    endcase
  end
  always @(*) begin
    case(stateMachine_state)
      `UartCtrlRxState_defaultEncoding_IDLE : stateMachine_state_string = "IDLE  ";
      `UartCtrlRxState_defaultEncoding_START : stateMachine_state_string = "START ";
      `UartCtrlRxState_defaultEncoding_DATA : stateMachine_state_string = "DATA  ";
      `UartCtrlRxState_defaultEncoding_PARITY : stateMachine_state_string = "PARITY";
      `UartCtrlRxState_defaultEncoding_STOP : stateMachine_state_string = "STOP  ";
      default : stateMachine_state_string = "??????";
    endcase
  end
  `endif

  assign _zz_1_ = 1'b0;
  assign sampler_synchroniser = bufferCC_4__io_dataOut;
  assign sampler_samples_0 = sampler_synchroniser;
  always @ (*) begin
    bitTimer_tick = 1'b0;
    if(sampler_tick)begin
      if(_zz_2_)begin
        bitTimer_tick = 1'b1;
      end
    end
  end

  assign io_read_valid = stateMachine_validReg;
  assign io_read_payload = stateMachine_shifter;
  always @ (posedge clk_12M) begin
    if(clockCtrl_systemReset) begin
      sampler_samples_1 <= 1'b1;
      sampler_samples_2 <= 1'b1;
      sampler_value <= 1'b1;
      sampler_tick <= 1'b0;
      stateMachine_state <= `UartCtrlRxState_defaultEncoding_IDLE;
      stateMachine_validReg <= 1'b0;
    end else begin
      if(io_samplingTick)begin
        sampler_samples_1 <= sampler_samples_0;
      end
      if(io_samplingTick)begin
        sampler_samples_2 <= sampler_samples_1;
      end
      sampler_value <= (((1'b0 || ((1'b1 && sampler_samples_0) && sampler_samples_1)) || ((1'b1 && sampler_samples_0) && sampler_samples_2)) || ((1'b1 && sampler_samples_1) && sampler_samples_2));
      sampler_tick <= io_samplingTick;
      stateMachine_validReg <= 1'b0;
      case(stateMachine_state)
        `UartCtrlRxState_defaultEncoding_IDLE : begin
          if(_zz_3_)begin
            stateMachine_state <= `UartCtrlRxState_defaultEncoding_START;
          end
        end
        `UartCtrlRxState_defaultEncoding_START : begin
          if(bitTimer_tick)begin
            stateMachine_state <= `UartCtrlRxState_defaultEncoding_DATA;
            if((sampler_value == 1'b1))begin
              stateMachine_state <= `UartCtrlRxState_defaultEncoding_IDLE;
            end
          end
        end
        `UartCtrlRxState_defaultEncoding_DATA : begin
          if(bitTimer_tick)begin
            if(_zz_4_)begin
              if((io_configFrame_parity == `UartParityType_defaultEncoding_NONE))begin
                stateMachine_state <= `UartCtrlRxState_defaultEncoding_STOP;
                stateMachine_validReg <= 1'b1;
              end else begin
                stateMachine_state <= `UartCtrlRxState_defaultEncoding_PARITY;
              end
            end
          end
        end
        `UartCtrlRxState_defaultEncoding_PARITY : begin
          if(bitTimer_tick)begin
            if((stateMachine_parity == sampler_value))begin
              stateMachine_state <= `UartCtrlRxState_defaultEncoding_STOP;
              stateMachine_validReg <= 1'b1;
            end else begin
              stateMachine_state <= `UartCtrlRxState_defaultEncoding_IDLE;
            end
          end
        end
        default : begin
          if(bitTimer_tick)begin
            if((! sampler_value))begin
              stateMachine_state <= `UartCtrlRxState_defaultEncoding_IDLE;
            end else begin
              if((bitCounter_value == _zz_6_))begin
                stateMachine_state <= `UartCtrlRxState_defaultEncoding_IDLE;
              end
            end
          end
        end
      endcase
    end
  end

  always @ (posedge clk_12M) begin
    if(sampler_tick)begin
      bitTimer_counter <= (bitTimer_counter - (3'b001));
      if(_zz_2_)begin
        bitTimer_counter <= (3'b100);
      end
    end
    if(bitTimer_tick)begin
      bitCounter_value <= (bitCounter_value + (3'b001));
    end
    if(bitTimer_tick)begin
      stateMachine_parity <= (stateMachine_parity ^ sampler_value);
    end
    case(stateMachine_state)
      `UartCtrlRxState_defaultEncoding_IDLE : begin
        if(_zz_3_)begin
          bitTimer_counter <= (3'b001);
        end
      end
      `UartCtrlRxState_defaultEncoding_START : begin
        if(bitTimer_tick)begin
          bitCounter_value <= (3'b000);
          stateMachine_parity <= (io_configFrame_parity == `UartParityType_defaultEncoding_ODD);
        end
      end
      `UartCtrlRxState_defaultEncoding_DATA : begin
        if(bitTimer_tick)begin
          stateMachine_shifter[bitCounter_value] <= sampler_value;
          if(_zz_4_)begin
            bitCounter_value <= (3'b000);
          end
        end
      end
      `UartCtrlRxState_defaultEncoding_PARITY : begin
        if(bitTimer_tick)begin
          bitCounter_value <= (3'b000);
        end
      end
      default : begin
      end
    endcase
  end

endmodule

module BufferCC_1_ (
      input   io_dataIn,
      output  io_dataOut,
      input   clk_12M,
      input   clockCtrl_resetUnbuffered_regNext);
  reg  buffers_0;
  reg  buffers_1;
  assign io_dataOut = buffers_1;
  always @ (posedge clk_12M) begin
    buffers_0 <= io_dataIn;
    buffers_1 <= buffers_0;
  end

endmodule

module UartCtrl (
      input  [2:0] io_config_frame_dataLength,
      input  `UartStopType_defaultEncoding_type io_config_frame_stop,
      input  `UartParityType_defaultEncoding_type io_config_frame_parity,
      input  [11:0] io_config_clockDivider,
      input   io_write_valid,
      output  io_write_ready,
      input  [7:0] io_write_payload,
      output  io_read_valid,
      output [7:0] io_read_payload,
      output  io_uart_txd,
      input   io_uart_rxd,
      input   clk_12M,
      input   clockCtrl_systemReset);
  wire  tx_io_write_ready;
  wire  tx_io_txd;
  wire  rx_io_read_valid;
  wire [7:0] rx_io_read_payload;
  reg [11:0] clockDivider_counter;
  wire  clockDivider_tick;
  `ifndef SYNTHESIS
  reg [23:0] io_config_frame_stop_string;
  reg [31:0] io_config_frame_parity_string;
  `endif

  UartCtrlTx tx ( 
    .io_configFrame_dataLength(io_config_frame_dataLength),
    .io_configFrame_stop(io_config_frame_stop),
    .io_configFrame_parity(io_config_frame_parity),
    .io_samplingTick(clockDivider_tick),
    .io_write_valid(io_write_valid),
    .io_write_ready(tx_io_write_ready),
    .io_write_payload(io_write_payload),
    .io_txd(tx_io_txd),
    .clk_12M(clk_12M),
    .clockCtrl_systemReset(clockCtrl_systemReset) 
  );
  UartCtrlRx rx ( 
    .io_configFrame_dataLength(io_config_frame_dataLength),
    .io_configFrame_stop(io_config_frame_stop),
    .io_configFrame_parity(io_config_frame_parity),
    .io_samplingTick(clockDivider_tick),
    .io_read_valid(rx_io_read_valid),
    .io_read_payload(rx_io_read_payload),
    .io_rxd(io_uart_rxd),
    .clk_12M(clk_12M),
    .clockCtrl_systemReset(clockCtrl_systemReset) 
  );
  `ifndef SYNTHESIS
  always @(*) begin
    case(io_config_frame_stop)
      `UartStopType_defaultEncoding_ONE : io_config_frame_stop_string = "ONE";
      `UartStopType_defaultEncoding_TWO : io_config_frame_stop_string = "TWO";
      default : io_config_frame_stop_string = "???";
    endcase
  end
  always @(*) begin
    case(io_config_frame_parity)
      `UartParityType_defaultEncoding_NONE : io_config_frame_parity_string = "NONE";
      `UartParityType_defaultEncoding_EVEN : io_config_frame_parity_string = "EVEN";
      `UartParityType_defaultEncoding_ODD : io_config_frame_parity_string = "ODD ";
      default : io_config_frame_parity_string = "????";
    endcase
  end
  `endif

  assign clockDivider_tick = (clockDivider_counter == (12'b000000000000));
  assign io_write_ready = tx_io_write_ready;
  assign io_read_valid = rx_io_read_valid;
  assign io_read_payload = rx_io_read_payload;
  assign io_uart_txd = tx_io_txd;
  always @ (posedge clk_12M) begin
    if(clockCtrl_systemReset) begin
      clockDivider_counter <= (12'b000000000000);
    end else begin
      clockDivider_counter <= (clockDivider_counter - (12'b000000000001));
      if(clockDivider_tick)begin
        clockDivider_counter <= io_config_clockDivider;
      end
    end
  end

endmodule

module StreamFifo (
      input   io_push_valid,
      output  io_push_ready,
      input  [7:0] io_push_payload,
      output  io_pop_valid,
      input   io_pop_ready,
      output [7:0] io_pop_payload,
      input   io_flush,
      output [0:0] io_occupancy,
      output [0:0] io_availability,
      input   clk_12M,
      input   clockCtrl_systemReset);
  wire  io_push_m2sPipe_valid;
  wire  io_push_m2sPipe_ready;
  wire [7:0] io_push_m2sPipe_payload;
  reg  _zz_1_;
  reg [7:0] _zz_2_;
  assign io_push_ready = ((1'b1 && (! io_push_m2sPipe_valid)) || io_push_m2sPipe_ready);
  assign io_push_m2sPipe_valid = _zz_1_;
  assign io_push_m2sPipe_payload = _zz_2_;
  assign io_pop_valid = io_push_m2sPipe_valid;
  assign io_push_m2sPipe_ready = io_pop_ready;
  assign io_pop_payload = io_push_m2sPipe_payload;
  assign io_occupancy = io_pop_valid;
  assign io_availability = (! io_pop_valid);
  always @ (posedge clk_12M) begin
    if(clockCtrl_systemReset) begin
      _zz_1_ <= 1'b0;
    end else begin
      if(io_push_ready)begin
        _zz_1_ <= io_push_valid;
      end
      if(io_flush)begin
        _zz_1_ <= 1'b0;
      end
    end
  end

  always @ (posedge clk_12M) begin
    if(io_push_ready)begin
      _zz_2_ <= io_push_payload;
    end
  end

endmodule


//StreamFifo_1_ remplaced by StreamFifo

module BufferCC_2_ (
      input  [3:0] io_dataIn,
      output [3:0] io_dataOut,
      input   clk_12M,
      input   clockCtrl_systemReset);
  reg [3:0] buffers_0;
  reg [3:0] buffers_1;
  assign io_dataOut = buffers_1;
  always @ (posedge clk_12M) begin
    buffers_0 <= io_dataIn;
    buffers_1 <= buffers_0;
  end

endmodule

module InstructionCache (
      input   io_flush,
      input   io_cpu_prefetch_isValid,
      output reg  io_cpu_prefetch_haltIt,
      input  [31:0] io_cpu_prefetch_pc,
      input   io_cpu_fetch_isValid,
      input   io_cpu_fetch_isStuck,
      input   io_cpu_fetch_isRemoved,
      input  [31:0] io_cpu_fetch_pc,
      output [31:0] io_cpu_fetch_data,
      input   io_cpu_fetch_dataBypassValid,
      input  [31:0] io_cpu_fetch_dataBypass,
      output  io_cpu_fetch_mmuBus_cmd_isValid,
      output [31:0] io_cpu_fetch_mmuBus_cmd_virtualAddress,
      output  io_cpu_fetch_mmuBus_cmd_bypassTranslation,
      input  [31:0] io_cpu_fetch_mmuBus_rsp_physicalAddress,
      input   io_cpu_fetch_mmuBus_rsp_isIoAccess,
      input   io_cpu_fetch_mmuBus_rsp_allowRead,
      input   io_cpu_fetch_mmuBus_rsp_allowWrite,
      input   io_cpu_fetch_mmuBus_rsp_allowExecute,
      input   io_cpu_fetch_mmuBus_rsp_exception,
      input   io_cpu_fetch_mmuBus_rsp_refilling,
      output  io_cpu_fetch_mmuBus_end,
      input   io_cpu_fetch_mmuBus_busy,
      output [31:0] io_cpu_fetch_physicalAddress,
      output  io_cpu_fetch_cacheMiss,
      output  io_cpu_fetch_error,
      output  io_cpu_fetch_mmuRefilling,
      output  io_cpu_fetch_mmuException,
      input   io_cpu_fetch_isUser,
      output  io_cpu_fetch_haltIt,
      input   io_cpu_decode_isValid,
      input   io_cpu_decode_isStuck,
      input  [31:0] io_cpu_decode_pc,
      output [31:0] io_cpu_decode_physicalAddress,
      output [31:0] io_cpu_decode_data,
      input   io_cpu_fill_valid,
      input  [31:0] io_cpu_fill_payload,
      output  io_mem_cmd_valid,
      input   io_mem_cmd_ready,
      output [31:0] io_mem_cmd_payload_address,
      output [2:0] io_mem_cmd_payload_size,
      input   io_mem_rsp_valid,
      input  [31:0] io_mem_rsp_payload_data,
      input   io_mem_rsp_payload_error,
      input   clk_12M,
      input   clockCtrl_systemReset);
  reg [21:0] _zz_10_;
  reg [31:0] _zz_11_;
  wire  _zz_12_;
  wire  _zz_13_;
  wire [0:0] _zz_14_;
  wire [0:0] _zz_15_;
  wire [21:0] _zz_16_;
  reg  _zz_1_;
  reg  _zz_2_;
  reg  lineLoader_fire;
  reg  lineLoader_valid;
  reg [31:0] lineLoader_address;
  reg  lineLoader_hadError;
  reg  lineLoader_flushPending;
  reg [7:0] lineLoader_flushCounter;
  reg  _zz_3_;
  reg  lineLoader_cmdSent;
  reg  lineLoader_wayToAllocate_willIncrement;
  wire  lineLoader_wayToAllocate_willClear;
  wire  lineLoader_wayToAllocate_willOverflowIfInc;
  wire  lineLoader_wayToAllocate_willOverflow;
  reg [2:0] lineLoader_wordIndex;
  wire  lineLoader_write_tag_0_valid;
  wire [6:0] lineLoader_write_tag_0_payload_address;
  wire  lineLoader_write_tag_0_payload_data_valid;
  wire  lineLoader_write_tag_0_payload_data_error;
  wire [19:0] lineLoader_write_tag_0_payload_data_address;
  wire  lineLoader_write_data_0_valid;
  wire [9:0] lineLoader_write_data_0_payload_address;
  wire [31:0] lineLoader_write_data_0_payload_data;
  wire  _zz_4_;
  wire [6:0] _zz_5_;
  wire  _zz_6_;
  wire  fetchStage_read_waysValues_0_tag_valid;
  wire  fetchStage_read_waysValues_0_tag_error;
  wire [19:0] fetchStage_read_waysValues_0_tag_address;
  wire [21:0] _zz_7_;
  wire [9:0] _zz_8_;
  wire  _zz_9_;
  wire [31:0] fetchStage_read_waysValues_0_data;
  wire  fetchStage_hit_hits_0;
  wire  fetchStage_hit_valid;
  wire  fetchStage_hit_error;
  wire [31:0] fetchStage_hit_data;
  wire [31:0] fetchStage_hit_word;
  reg [21:0] ways_0_tags [0:127];
  reg [31:0] ways_0_datas [0:1023];
  assign _zz_12_ = (! lineLoader_flushCounter[7]);
  assign _zz_13_ = (lineLoader_flushPending && (! (lineLoader_valid || io_cpu_fetch_isValid)));
  assign _zz_14_ = _zz_7_[0 : 0];
  assign _zz_15_ = _zz_7_[1 : 1];
  assign _zz_16_ = {lineLoader_write_tag_0_payload_data_address,{lineLoader_write_tag_0_payload_data_error,lineLoader_write_tag_0_payload_data_valid}};
  always @ (posedge clk_12M) begin
    if(_zz_2_) begin
      ways_0_tags[lineLoader_write_tag_0_payload_address] <= _zz_16_;
    end
  end

  always @ (posedge clk_12M) begin
    if(_zz_6_) begin
      _zz_10_ <= ways_0_tags[_zz_5_];
    end
  end

  always @ (posedge clk_12M) begin
    if(_zz_1_) begin
      ways_0_datas[lineLoader_write_data_0_payload_address] <= lineLoader_write_data_0_payload_data;
    end
  end

  always @ (posedge clk_12M) begin
    if(_zz_9_) begin
      _zz_11_ <= ways_0_datas[_zz_8_];
    end
  end

  always @ (*) begin
    _zz_1_ = 1'b0;
    if(lineLoader_write_data_0_valid)begin
      _zz_1_ = 1'b1;
    end
  end

  always @ (*) begin
    _zz_2_ = 1'b0;
    if(lineLoader_write_tag_0_valid)begin
      _zz_2_ = 1'b1;
    end
  end

  assign io_cpu_fetch_haltIt = io_cpu_fetch_mmuBus_busy;
  always @ (*) begin
    lineLoader_fire = 1'b0;
    if(io_mem_rsp_valid)begin
      if((lineLoader_wordIndex == (3'b111)))begin
        lineLoader_fire = 1'b1;
      end
    end
  end

  always @ (*) begin
    io_cpu_prefetch_haltIt = (lineLoader_valid || lineLoader_flushPending);
    if(_zz_12_)begin
      io_cpu_prefetch_haltIt = 1'b1;
    end
    if((! _zz_3_))begin
      io_cpu_prefetch_haltIt = 1'b1;
    end
    if(io_flush)begin
      io_cpu_prefetch_haltIt = 1'b1;
    end
  end

  assign io_mem_cmd_valid = (lineLoader_valid && (! lineLoader_cmdSent));
  assign io_mem_cmd_payload_address = {lineLoader_address[31 : 5],(5'b00000)};
  assign io_mem_cmd_payload_size = (3'b101);
  always @ (*) begin
    lineLoader_wayToAllocate_willIncrement = 1'b0;
    if((! lineLoader_valid))begin
      lineLoader_wayToAllocate_willIncrement = 1'b1;
    end
  end

  assign lineLoader_wayToAllocate_willClear = 1'b0;
  assign lineLoader_wayToAllocate_willOverflowIfInc = 1'b1;
  assign lineLoader_wayToAllocate_willOverflow = (lineLoader_wayToAllocate_willOverflowIfInc && lineLoader_wayToAllocate_willIncrement);
  assign _zz_4_ = 1'b1;
  assign lineLoader_write_tag_0_valid = ((_zz_4_ && lineLoader_fire) || (! lineLoader_flushCounter[7]));
  assign lineLoader_write_tag_0_payload_address = (lineLoader_flushCounter[7] ? lineLoader_address[11 : 5] : lineLoader_flushCounter[6 : 0]);
  assign lineLoader_write_tag_0_payload_data_valid = lineLoader_flushCounter[7];
  assign lineLoader_write_tag_0_payload_data_error = (lineLoader_hadError || io_mem_rsp_payload_error);
  assign lineLoader_write_tag_0_payload_data_address = lineLoader_address[31 : 12];
  assign lineLoader_write_data_0_valid = (io_mem_rsp_valid && _zz_4_);
  assign lineLoader_write_data_0_payload_address = {lineLoader_address[11 : 5],lineLoader_wordIndex};
  assign lineLoader_write_data_0_payload_data = io_mem_rsp_payload_data;
  assign _zz_5_ = io_cpu_prefetch_pc[11 : 5];
  assign _zz_6_ = (! io_cpu_fetch_isStuck);
  assign _zz_7_ = _zz_10_;
  assign fetchStage_read_waysValues_0_tag_valid = _zz_14_[0];
  assign fetchStage_read_waysValues_0_tag_error = _zz_15_[0];
  assign fetchStage_read_waysValues_0_tag_address = _zz_7_[21 : 2];
  assign _zz_8_ = io_cpu_prefetch_pc[11 : 2];
  assign _zz_9_ = (! io_cpu_fetch_isStuck);
  assign fetchStage_read_waysValues_0_data = _zz_11_;
  assign fetchStage_hit_hits_0 = (fetchStage_read_waysValues_0_tag_valid && (fetchStage_read_waysValues_0_tag_address == io_cpu_fetch_mmuBus_rsp_physicalAddress[31 : 12]));
  assign fetchStage_hit_valid = (fetchStage_hit_hits_0 != (1'b0));
  assign fetchStage_hit_error = fetchStage_read_waysValues_0_tag_error;
  assign fetchStage_hit_data = fetchStage_read_waysValues_0_data;
  assign fetchStage_hit_word = fetchStage_hit_data[31 : 0];
  assign io_cpu_fetch_data = (io_cpu_fetch_dataBypassValid ? io_cpu_fetch_dataBypass : fetchStage_hit_word);
  assign io_cpu_fetch_mmuBus_cmd_isValid = io_cpu_fetch_isValid;
  assign io_cpu_fetch_mmuBus_cmd_virtualAddress = io_cpu_fetch_pc;
  assign io_cpu_fetch_mmuBus_cmd_bypassTranslation = 1'b0;
  assign io_cpu_fetch_mmuBus_end = ((! io_cpu_fetch_isStuck) || io_cpu_fetch_isRemoved);
  assign io_cpu_fetch_physicalAddress = io_cpu_fetch_mmuBus_rsp_physicalAddress;
  assign io_cpu_fetch_cacheMiss = (! fetchStage_hit_valid);
  assign io_cpu_fetch_error = fetchStage_hit_error;
  assign io_cpu_fetch_mmuRefilling = io_cpu_fetch_mmuBus_rsp_refilling;
  assign io_cpu_fetch_mmuException = ((! io_cpu_fetch_mmuBus_rsp_refilling) && (io_cpu_fetch_mmuBus_rsp_exception || (! io_cpu_fetch_mmuBus_rsp_allowExecute)));
  always @ (posedge clk_12M) begin
    if(clockCtrl_systemReset) begin
      lineLoader_valid <= 1'b0;
      lineLoader_hadError <= 1'b0;
      lineLoader_flushPending <= 1'b1;
      lineLoader_cmdSent <= 1'b0;
      lineLoader_wordIndex <= (3'b000);
    end else begin
      if(lineLoader_fire)begin
        lineLoader_valid <= 1'b0;
      end
      if(lineLoader_fire)begin
        lineLoader_hadError <= 1'b0;
      end
      if(io_cpu_fill_valid)begin
        lineLoader_valid <= 1'b1;
      end
      if(io_flush)begin
        lineLoader_flushPending <= 1'b1;
      end
      if(_zz_13_)begin
        lineLoader_flushPending <= 1'b0;
      end
      if((io_mem_cmd_valid && io_mem_cmd_ready))begin
        lineLoader_cmdSent <= 1'b1;
      end
      if(lineLoader_fire)begin
        lineLoader_cmdSent <= 1'b0;
      end
      if(io_mem_rsp_valid)begin
        lineLoader_wordIndex <= (lineLoader_wordIndex + (3'b001));
        if(io_mem_rsp_payload_error)begin
          lineLoader_hadError <= 1'b1;
        end
      end
    end
  end

  always @ (posedge clk_12M) begin
    if(io_cpu_fill_valid)begin
      lineLoader_address <= io_cpu_fill_payload;
    end
    if(_zz_12_)begin
      lineLoader_flushCounter <= (lineLoader_flushCounter + (8'b00000001));
    end
    _zz_3_ <= lineLoader_flushCounter[7];
    if(_zz_13_)begin
      lineLoader_flushCounter <= (8'b00000000);
    end
  end

endmodule

module FlowCCByToggle (
      input   io_input_valid,
      input   io_input_payload_last,
      input  [0:0] io_input_payload_fragment,
      output  io_output_valid,
      output  io_output_payload_last,
      output [0:0] io_output_payload_fragment,
      input   io_jtag_tck,
      input   clk_12M,
      input   clockCtrl_resetUnbuffered_regNext);
  wire  bufferCC_4__io_dataOut;
  wire  outHitSignal;
  reg  inputArea_target = 0;
  reg  inputArea_data_last;
  reg [0:0] inputArea_data_fragment;
  wire  outputArea_target;
  reg  outputArea_hit;
  wire  outputArea_flow_valid;
  wire  outputArea_flow_payload_last;
  wire [0:0] outputArea_flow_payload_fragment;
  reg  outputArea_flow_m2sPipe_valid;
  reg  outputArea_flow_m2sPipe_payload_last;
  reg [0:0] outputArea_flow_m2sPipe_payload_fragment;
  BufferCC_1_ bufferCC_4_ ( 
    .io_dataIn(inputArea_target),
    .io_dataOut(bufferCC_4__io_dataOut),
    .clk_12M(clk_12M),
    .clockCtrl_resetUnbuffered_regNext(clockCtrl_resetUnbuffered_regNext) 
  );
  assign outputArea_target = bufferCC_4__io_dataOut;
  assign outputArea_flow_valid = (outputArea_target != outputArea_hit);
  assign outputArea_flow_payload_last = inputArea_data_last;
  assign outputArea_flow_payload_fragment = inputArea_data_fragment;
  assign io_output_valid = outputArea_flow_m2sPipe_valid;
  assign io_output_payload_last = outputArea_flow_m2sPipe_payload_last;
  assign io_output_payload_fragment = outputArea_flow_m2sPipe_payload_fragment;
  always @ (posedge io_jtag_tck) begin
    if(io_input_valid)begin
      inputArea_target <= (! inputArea_target);
      inputArea_data_last <= io_input_payload_last;
      inputArea_data_fragment <= io_input_payload_fragment;
    end
  end

  always @ (posedge clk_12M) begin
    outputArea_hit <= outputArea_target;
    if(outputArea_flow_valid)begin
      outputArea_flow_m2sPipe_payload_last <= outputArea_flow_payload_last;
      outputArea_flow_m2sPipe_payload_fragment <= outputArea_flow_payload_fragment;
    end
  end

  always @ (posedge clk_12M) begin
    if(clockCtrl_resetUnbuffered_regNext) begin
      outputArea_flow_m2sPipe_valid <= 1'b0;
    end else begin
      outputArea_flow_m2sPipe_valid <= outputArea_flow_valid;
    end
  end

endmodule

module SpiXdrMasterCtrl (
      input   io_config_kind_cpol,
      input   io_config_kind_cpha,
      input  [7:0] io_config_sclkToogle,
      input  [0:0] io_config_mod,
      input  [0:0] io_config_ss_activeHigh,
      input  [7:0] io_config_ss_setup,
      input  [7:0] io_config_ss_hold,
      input  [7:0] io_config_ss_disable,
      input   io_cmd_valid,
      output reg  io_cmd_ready,
      input   io_cmd_payload_kind,
      input   io_cmd_payload_read,
      input   io_cmd_payload_write,
      input  [7:0] io_cmd_payload_data,
      output  io_rsp_valid,
      output [7:0] io_rsp_payload_data,
      output [1:0] io_spi_sclk_write,
      output reg  io_spi_data_0_writeEnable,
      input  [1:0] io_spi_data_0_read,
      output reg [1:0] io_spi_data_0_write,
      output reg  io_spi_data_1_writeEnable,
      input  [1:0] io_spi_data_1_read,
      output reg [1:0] io_spi_data_1_write,
      output [0:0] io_spi_ss,
      input   clk_12M,
      input   clockCtrl_systemReset);
  reg [0:0] _zz_3_;
  reg [1:0] _zz_4_;
  wire  _zz_5_;
  wire  _zz_6_;
  wire  _zz_7_;
  wire  _zz_8_;
  wire  _zz_9_;
  wire  _zz_10_;
  wire  _zz_11_;
  wire [2:0] _zz_12_;
  wire [2:0] _zz_13_;
  wire [1:0] _zz_14_;
  wire [8:0] _zz_15_;
  reg [7:0] timer_counter;
  reg  timer_reset;
  wire  timer_ss_setupHit;
  wire  timer_ss_holdHit;
  wire  timer_ss_disableHit;
  wire  timer_sclkToogleHit;
  reg  fsm_state;
  reg [2:0] fsm_counter;
  reg [1:0] _zz_1_;
  wire [2:0] fsm_counterPlus;
  reg  fsm_fastRate;
  reg  fsm_isDdr;
  reg  fsm_readFill;
  reg  fsm_readDone;
  reg [0:0] fsm_ss;
  reg [1:0] outputPhy_sclkWrite;
  reg [1:0] _zz_2_;
  reg [1:0] outputPhy_dataWrite;
  reg [1:0] outputPhy_widthSel;
  reg [0:0] io_config_mod_delay_1;
  reg [0:0] inputPhy_mod;
  reg  fsm_readFill_delay_1;
  reg  inputPhy_readFill;
  reg  fsm_readDone_delay_1;
  reg  inputPhy_readDone;
  reg [6:0] inputPhy_buffer;
  reg [7:0] inputPhy_bufferNext;
  reg [1:0] inputPhy_widthSel;
  wire [1:0] inputPhy_dataWrite;
  reg [1:0] inputPhy_dataRead;
  reg  fsm_state_delay_1;
  reg  fsm_state_delay_2;
  reg [1:0] inputPhy_dataReadBuffer;
  assign _zz_5_ = (! io_cmd_payload_kind);
  assign _zz_6_ = io_cmd_payload_data[7];
  assign _zz_7_ = (! fsm_state);
  assign _zz_8_ = ((! io_cmd_valid) || io_cmd_ready);
  assign _zz_9_ = ((timer_sclkToogleHit && (fsm_state || fsm_isDdr)) || fsm_fastRate);
  assign _zz_10_ = (fsm_counterPlus == (3'b000));
  assign _zz_11_ = (io_cmd_valid && io_cmd_payload_write);
  assign _zz_12_ = {1'd0, _zz_1_};
  assign _zz_13_ = (fsm_counter >>> 0);
  assign _zz_14_ = (fsm_counter >>> 1);
  assign _zz_15_ = {inputPhy_buffer,inputPhy_dataRead[1 : 0]};
  always @(*) begin
    case(_zz_13_)
      3'b000 : begin
        _zz_3_ = io_cmd_payload_data[7 : 7];
      end
      3'b001 : begin
        _zz_3_ = io_cmd_payload_data[6 : 6];
      end
      3'b010 : begin
        _zz_3_ = io_cmd_payload_data[5 : 5];
      end
      3'b011 : begin
        _zz_3_ = io_cmd_payload_data[4 : 4];
      end
      3'b100 : begin
        _zz_3_ = io_cmd_payload_data[3 : 3];
      end
      3'b101 : begin
        _zz_3_ = io_cmd_payload_data[2 : 2];
      end
      3'b110 : begin
        _zz_3_ = io_cmd_payload_data[1 : 1];
      end
      default : begin
        _zz_3_ = io_cmd_payload_data[0 : 0];
      end
    endcase
  end

  always @(*) begin
    case(_zz_14_)
      2'b00 : begin
        _zz_4_ = io_cmd_payload_data[7 : 6];
      end
      2'b01 : begin
        _zz_4_ = io_cmd_payload_data[5 : 4];
      end
      2'b10 : begin
        _zz_4_ = io_cmd_payload_data[3 : 2];
      end
      default : begin
        _zz_4_ = io_cmd_payload_data[1 : 0];
      end
    endcase
  end

  always @ (*) begin
    timer_reset = 1'b0;
    if(io_cmd_valid)begin
      if(_zz_5_)begin
        timer_reset = timer_sclkToogleHit;
      end else begin
        if(! _zz_6_) begin
          if(_zz_7_)begin
            if(timer_ss_holdHit)begin
              timer_reset = 1'b1;
            end
          end
        end
      end
    end
    if(_zz_8_)begin
      timer_reset = 1'b1;
    end
  end

  assign timer_ss_setupHit = (timer_counter == io_config_ss_setup);
  assign timer_ss_holdHit = (timer_counter == io_config_ss_hold);
  assign timer_ss_disableHit = (timer_counter == io_config_ss_disable);
  assign timer_sclkToogleHit = (timer_counter == io_config_sclkToogle);
  always @ (*) begin
    _zz_1_ = (2'bxx);
    case(io_config_mod)
      1'b0 : begin
        _zz_1_ = (2'b01);
      end
      default : begin
        _zz_1_ = (2'b10);
      end
    endcase
  end

  assign fsm_counterPlus = (fsm_counter + _zz_12_);
  always @ (*) begin
    fsm_fastRate = 1'bx;
    case(io_config_mod)
      1'b0 : begin
        fsm_fastRate = 1'b1;
      end
      default : begin
        fsm_fastRate = 1'b1;
      end
    endcase
  end

  always @ (*) begin
    fsm_isDdr = 1'bx;
    case(io_config_mod)
      1'b0 : begin
        fsm_isDdr = 1'b0;
      end
      default : begin
        fsm_isDdr = 1'b0;
      end
    endcase
  end

  always @ (*) begin
    fsm_readFill = 1'b0;
    if(io_cmd_valid)begin
      if(_zz_5_)begin
        if(_zz_9_)begin
          fsm_readFill = 1'b1;
        end
      end
    end
  end

  always @ (*) begin
    fsm_readDone = 1'b0;
    if(io_cmd_valid)begin
      if(_zz_5_)begin
        if(_zz_9_)begin
          if(_zz_10_)begin
            fsm_readDone = io_cmd_payload_read;
          end
        end
      end
    end
  end

  assign io_spi_ss = (~ (fsm_ss ^ io_config_ss_activeHigh));
  always @ (*) begin
    io_cmd_ready = 1'b0;
    if(io_cmd_valid)begin
      if(_zz_5_)begin
        if(_zz_9_)begin
          if(_zz_10_)begin
            io_cmd_ready = 1'b1;
          end
        end
      end else begin
        if(_zz_6_)begin
          if(timer_ss_setupHit)begin
            io_cmd_ready = 1'b1;
          end
        end else begin
          if(! _zz_7_) begin
            if(timer_ss_disableHit)begin
              io_cmd_ready = 1'b1;
            end
          end
        end
      end
    end
  end

  always @ (*) begin
    outputPhy_sclkWrite = (2'b00);
    if((io_cmd_valid && (! io_cmd_payload_kind)))begin
      case(io_config_mod)
        1'b0 : begin
          outputPhy_sclkWrite[0] = io_config_kind_cpha;
          outputPhy_sclkWrite[1] = (! io_config_kind_cpha);
        end
        default : begin
          outputPhy_sclkWrite[0] = io_config_kind_cpha;
          outputPhy_sclkWrite[1] = (! io_config_kind_cpha);
        end
      endcase
    end
  end

  always @ (*) begin
    _zz_2_[0] = io_config_kind_cpol;
    _zz_2_[1] = io_config_kind_cpol;
  end

  assign io_spi_sclk_write = (outputPhy_sclkWrite ^ _zz_2_);
  always @ (*) begin
    outputPhy_widthSel = (2'bxx);
    case(io_config_mod)
      1'b0 : begin
        outputPhy_widthSel = (2'b00);
      end
      default : begin
        outputPhy_widthSel = (2'b01);
      end
    endcase
  end

  always @ (*) begin
    outputPhy_dataWrite = (2'bxx);
    case(outputPhy_widthSel)
      2'b00 : begin
        outputPhy_dataWrite[0 : 0] = _zz_3_;
      end
      2'b01 : begin
        outputPhy_dataWrite[1 : 0] = _zz_4_;
      end
      default : begin
      end
    endcase
  end

  always @ (*) begin
    io_spi_data_0_writeEnable = 1'b0;
    case(io_config_mod)
      1'b0 : begin
        if((io_cmd_valid && io_cmd_payload_write))begin
          io_spi_data_0_writeEnable = 1'b1;
        end
      end
      default : begin
        if(_zz_11_)begin
          io_spi_data_0_writeEnable = 1'b1;
        end
      end
    endcase
  end

  always @ (*) begin
    io_spi_data_1_writeEnable = 1'b0;
    case(io_config_mod)
      1'b0 : begin
      end
      default : begin
        if(_zz_11_)begin
          io_spi_data_1_writeEnable = 1'b1;
        end
      end
    endcase
  end

  always @ (*) begin
    io_spi_data_0_write = (2'bxx);
    case(io_config_mod)
      1'b0 : begin
        io_spi_data_0_write[0] = outputPhy_dataWrite[0];
        io_spi_data_0_write[1] = outputPhy_dataWrite[0];
      end
      default : begin
        io_spi_data_0_write[0] = outputPhy_dataWrite[0];
        io_spi_data_0_write[1] = outputPhy_dataWrite[0];
      end
    endcase
  end

  always @ (*) begin
    io_spi_data_1_write = (2'bxx);
    case(io_config_mod)
      1'b0 : begin
      end
      default : begin
        io_spi_data_1_write[0] = outputPhy_dataWrite[1];
        io_spi_data_1_write[1] = outputPhy_dataWrite[1];
      end
    endcase
  end

  always @ (*) begin
    inputPhy_bufferNext = (8'bxxxxxxxx);
    case(inputPhy_widthSel)
      2'b00 : begin
        inputPhy_bufferNext = {inputPhy_buffer,inputPhy_dataRead[0 : 0]};
      end
      2'b01 : begin
        inputPhy_bufferNext = _zz_15_[7:0];
      end
      default : begin
      end
    endcase
  end

  always @ (*) begin
    inputPhy_widthSel = (2'bxx);
    case(inputPhy_mod)
      1'b0 : begin
        inputPhy_widthSel = (2'b00);
      end
      default : begin
        inputPhy_widthSel = (2'b01);
      end
    endcase
  end

  always @ (*) begin
    inputPhy_dataRead = (2'bxx);
    case(inputPhy_mod)
      1'b0 : begin
        inputPhy_dataRead[0] = io_spi_data_1_read[1];
      end
      default : begin
        inputPhy_dataRead[0] = io_spi_data_0_read[1];
        inputPhy_dataRead[1] = io_spi_data_1_read[1];
      end
    endcase
  end

  assign io_rsp_valid = inputPhy_readDone;
  assign io_rsp_payload_data = inputPhy_bufferNext;
  always @ (posedge clk_12M) begin
    timer_counter <= (timer_counter + (8'b00000001));
    if(timer_reset)begin
      timer_counter <= (8'b00000000);
    end
    io_config_mod_delay_1 <= io_config_mod;
    inputPhy_mod <= io_config_mod_delay_1;
    fsm_state_delay_1 <= fsm_state;
    fsm_state_delay_2 <= fsm_state_delay_1;
    if((! fsm_state_delay_2))begin
      inputPhy_dataReadBuffer <= {io_spi_data_1_read[0],io_spi_data_0_read[0]};
    end
    case(inputPhy_widthSel)
      2'b00 : begin
        if(inputPhy_readFill)begin
          inputPhy_buffer <= inputPhy_bufferNext[6:0];
        end
      end
      2'b01 : begin
        if(inputPhy_readFill)begin
          inputPhy_buffer <= inputPhy_bufferNext[6:0];
        end
      end
      default : begin
      end
    endcase
  end

  always @ (posedge clk_12M) begin
    if(clockCtrl_systemReset) begin
      fsm_state <= 1'b0;
      fsm_counter <= (3'b000);
      fsm_ss <= (1'b0);
      fsm_readFill_delay_1 <= 1'b0;
      inputPhy_readFill <= 1'b0;
      fsm_readDone_delay_1 <= 1'b0;
      inputPhy_readDone <= 1'b0;
    end else begin
      if(io_cmd_valid)begin
        if(_zz_5_)begin
          if(timer_sclkToogleHit)begin
            fsm_state <= (! fsm_state);
          end
          if(_zz_9_)begin
            fsm_counter <= fsm_counterPlus;
            if(_zz_10_)begin
              fsm_state <= 1'b0;
            end
          end
        end else begin
          if(_zz_6_)begin
            fsm_ss[0] <= 1'b1;
          end else begin
            if(_zz_7_)begin
              if(timer_ss_holdHit)begin
                fsm_state <= 1'b1;
              end
            end else begin
              fsm_ss[0] <= 1'b0;
            end
          end
        end
      end
      if(_zz_8_)begin
        fsm_state <= 1'b0;
        fsm_counter <= (3'b000);
      end
      fsm_readFill_delay_1 <= fsm_readFill;
      inputPhy_readFill <= fsm_readFill_delay_1;
      fsm_readDone_delay_1 <= fsm_readDone;
      inputPhy_readDone <= fsm_readDone_delay_1;
    end
  end

endmodule

module StreamFifo_2_ (
      input   io_push_valid,
      output  io_push_ready,
      input   io_push_payload_kind,
      input   io_push_payload_read,
      input   io_push_payload_write,
      input  [7:0] io_push_payload_data,
      output  io_pop_valid,
      input   io_pop_ready,
      output  io_pop_payload_kind,
      output  io_pop_payload_read,
      output  io_pop_payload_write,
      output [7:0] io_pop_payload_data,
      input   io_flush,
      output [6:0] io_occupancy,
      output [6:0] io_availability,
      input   clk_12M,
      input   clockCtrl_systemReset);
  reg [10:0] _zz_4_;
  wire [0:0] _zz_5_;
  wire [5:0] _zz_6_;
  wire [0:0] _zz_7_;
  wire [5:0] _zz_8_;
  wire [0:0] _zz_9_;
  wire [0:0] _zz_10_;
  wire [0:0] _zz_11_;
  wire [5:0] _zz_12_;
  wire  _zz_13_;
  wire [10:0] _zz_14_;
  reg  _zz_1_;
  reg  logic_pushPtr_willIncrement;
  reg  logic_pushPtr_willClear;
  reg [5:0] logic_pushPtr_valueNext;
  reg [5:0] logic_pushPtr_value;
  wire  logic_pushPtr_willOverflowIfInc;
  wire  logic_pushPtr_willOverflow;
  reg  logic_popPtr_willIncrement;
  reg  logic_popPtr_willClear;
  reg [5:0] logic_popPtr_valueNext;
  reg [5:0] logic_popPtr_value;
  wire  logic_popPtr_willOverflowIfInc;
  wire  logic_popPtr_willOverflow;
  wire  logic_ptrMatch;
  reg  logic_risingOccupancy;
  wire  logic_pushing;
  wire  logic_popping;
  wire  logic_empty;
  wire  logic_full;
  reg  _zz_2_;
  wire [10:0] _zz_3_;
  wire [5:0] logic_ptrDif;
  reg [10:0] logic_ram [0:63];
  assign _zz_5_ = logic_pushPtr_willIncrement;
  assign _zz_6_ = {5'd0, _zz_5_};
  assign _zz_7_ = logic_popPtr_willIncrement;
  assign _zz_8_ = {5'd0, _zz_7_};
  assign _zz_9_ = _zz_3_[0 : 0];
  assign _zz_10_ = _zz_3_[1 : 1];
  assign _zz_11_ = _zz_3_[2 : 2];
  assign _zz_12_ = (logic_popPtr_value - logic_pushPtr_value);
  assign _zz_13_ = 1'b1;
  assign _zz_14_ = {io_push_payload_data,{io_push_payload_write,{io_push_payload_read,io_push_payload_kind}}};
  always @ (posedge clk_12M) begin
    if(_zz_1_) begin
      logic_ram[logic_pushPtr_value] <= _zz_14_;
    end
  end

  always @ (posedge clk_12M) begin
    if(_zz_13_) begin
      _zz_4_ <= logic_ram[logic_popPtr_valueNext];
    end
  end

  always @ (*) begin
    _zz_1_ = 1'b0;
    if(logic_pushing)begin
      _zz_1_ = 1'b1;
    end
  end

  always @ (*) begin
    logic_pushPtr_willIncrement = 1'b0;
    if(logic_pushing)begin
      logic_pushPtr_willIncrement = 1'b1;
    end
  end

  always @ (*) begin
    logic_pushPtr_willClear = 1'b0;
    if(io_flush)begin
      logic_pushPtr_willClear = 1'b1;
    end
  end

  assign logic_pushPtr_willOverflowIfInc = (logic_pushPtr_value == (6'b111111));
  assign logic_pushPtr_willOverflow = (logic_pushPtr_willOverflowIfInc && logic_pushPtr_willIncrement);
  always @ (*) begin
    logic_pushPtr_valueNext = (logic_pushPtr_value + _zz_6_);
    if(logic_pushPtr_willClear)begin
      logic_pushPtr_valueNext = (6'b000000);
    end
  end

  always @ (*) begin
    logic_popPtr_willIncrement = 1'b0;
    if(logic_popping)begin
      logic_popPtr_willIncrement = 1'b1;
    end
  end

  always @ (*) begin
    logic_popPtr_willClear = 1'b0;
    if(io_flush)begin
      logic_popPtr_willClear = 1'b1;
    end
  end

  assign logic_popPtr_willOverflowIfInc = (logic_popPtr_value == (6'b111111));
  assign logic_popPtr_willOverflow = (logic_popPtr_willOverflowIfInc && logic_popPtr_willIncrement);
  always @ (*) begin
    logic_popPtr_valueNext = (logic_popPtr_value + _zz_8_);
    if(logic_popPtr_willClear)begin
      logic_popPtr_valueNext = (6'b000000);
    end
  end

  assign logic_ptrMatch = (logic_pushPtr_value == logic_popPtr_value);
  assign logic_pushing = (io_push_valid && io_push_ready);
  assign logic_popping = (io_pop_valid && io_pop_ready);
  assign logic_empty = (logic_ptrMatch && (! logic_risingOccupancy));
  assign logic_full = (logic_ptrMatch && logic_risingOccupancy);
  assign io_push_ready = (! logic_full);
  assign io_pop_valid = ((! logic_empty) && (! (_zz_2_ && (! logic_full))));
  assign _zz_3_ = _zz_4_;
  assign io_pop_payload_kind = _zz_9_[0];
  assign io_pop_payload_read = _zz_10_[0];
  assign io_pop_payload_write = _zz_11_[0];
  assign io_pop_payload_data = _zz_3_[10 : 3];
  assign logic_ptrDif = (logic_pushPtr_value - logic_popPtr_value);
  assign io_occupancy = {(logic_risingOccupancy && logic_ptrMatch),logic_ptrDif};
  assign io_availability = {((! logic_risingOccupancy) && logic_ptrMatch),_zz_12_};
  always @ (posedge clk_12M) begin
    if(clockCtrl_systemReset) begin
      logic_pushPtr_value <= (6'b000000);
      logic_popPtr_value <= (6'b000000);
      logic_risingOccupancy <= 1'b0;
      _zz_2_ <= 1'b0;
    end else begin
      logic_pushPtr_value <= logic_pushPtr_valueNext;
      logic_popPtr_value <= logic_popPtr_valueNext;
      _zz_2_ <= (logic_popPtr_valueNext == logic_pushPtr_value);
      if((logic_pushing != logic_popping))begin
        logic_risingOccupancy <= logic_pushing;
      end
      if(io_flush)begin
        logic_risingOccupancy <= 1'b0;
      end
    end
  end

endmodule

module StreamFifo_3_ (
      input   io_push_valid,
      output  io_push_ready,
      input  [7:0] io_push_payload_data,
      output  io_pop_valid,
      input   io_pop_ready,
      output [7:0] io_pop_payload_data,
      input   io_flush,
      output [6:0] io_occupancy,
      output [6:0] io_availability,
      input   clk_12M,
      input   clockCtrl_systemReset);
  reg [7:0] _zz_3_;
  wire [0:0] _zz_4_;
  wire [5:0] _zz_5_;
  wire [0:0] _zz_6_;
  wire [5:0] _zz_7_;
  wire [5:0] _zz_8_;
  wire  _zz_9_;
  reg  _zz_1_;
  reg  logic_pushPtr_willIncrement;
  reg  logic_pushPtr_willClear;
  reg [5:0] logic_pushPtr_valueNext;
  reg [5:0] logic_pushPtr_value;
  wire  logic_pushPtr_willOverflowIfInc;
  wire  logic_pushPtr_willOverflow;
  reg  logic_popPtr_willIncrement;
  reg  logic_popPtr_willClear;
  reg [5:0] logic_popPtr_valueNext;
  reg [5:0] logic_popPtr_value;
  wire  logic_popPtr_willOverflowIfInc;
  wire  logic_popPtr_willOverflow;
  wire  logic_ptrMatch;
  reg  logic_risingOccupancy;
  wire  logic_pushing;
  wire  logic_popping;
  wire  logic_empty;
  wire  logic_full;
  reg  _zz_2_;
  wire [5:0] logic_ptrDif;
  reg [7:0] logic_ram [0:63];
  assign _zz_4_ = logic_pushPtr_willIncrement;
  assign _zz_5_ = {5'd0, _zz_4_};
  assign _zz_6_ = logic_popPtr_willIncrement;
  assign _zz_7_ = {5'd0, _zz_6_};
  assign _zz_8_ = (logic_popPtr_value - logic_pushPtr_value);
  assign _zz_9_ = 1'b1;
  always @ (posedge clk_12M) begin
    if(_zz_1_) begin
      logic_ram[logic_pushPtr_value] <= io_push_payload_data;
    end
  end

  always @ (posedge clk_12M) begin
    if(_zz_9_) begin
      _zz_3_ <= logic_ram[logic_popPtr_valueNext];
    end
  end

  always @ (*) begin
    _zz_1_ = 1'b0;
    if(logic_pushing)begin
      _zz_1_ = 1'b1;
    end
  end

  always @ (*) begin
    logic_pushPtr_willIncrement = 1'b0;
    if(logic_pushing)begin
      logic_pushPtr_willIncrement = 1'b1;
    end
  end

  always @ (*) begin
    logic_pushPtr_willClear = 1'b0;
    if(io_flush)begin
      logic_pushPtr_willClear = 1'b1;
    end
  end

  assign logic_pushPtr_willOverflowIfInc = (logic_pushPtr_value == (6'b111111));
  assign logic_pushPtr_willOverflow = (logic_pushPtr_willOverflowIfInc && logic_pushPtr_willIncrement);
  always @ (*) begin
    logic_pushPtr_valueNext = (logic_pushPtr_value + _zz_5_);
    if(logic_pushPtr_willClear)begin
      logic_pushPtr_valueNext = (6'b000000);
    end
  end

  always @ (*) begin
    logic_popPtr_willIncrement = 1'b0;
    if(logic_popping)begin
      logic_popPtr_willIncrement = 1'b1;
    end
  end

  always @ (*) begin
    logic_popPtr_willClear = 1'b0;
    if(io_flush)begin
      logic_popPtr_willClear = 1'b1;
    end
  end

  assign logic_popPtr_willOverflowIfInc = (logic_popPtr_value == (6'b111111));
  assign logic_popPtr_willOverflow = (logic_popPtr_willOverflowIfInc && logic_popPtr_willIncrement);
  always @ (*) begin
    logic_popPtr_valueNext = (logic_popPtr_value + _zz_7_);
    if(logic_popPtr_willClear)begin
      logic_popPtr_valueNext = (6'b000000);
    end
  end

  assign logic_ptrMatch = (logic_pushPtr_value == logic_popPtr_value);
  assign logic_pushing = (io_push_valid && io_push_ready);
  assign logic_popping = (io_pop_valid && io_pop_ready);
  assign logic_empty = (logic_ptrMatch && (! logic_risingOccupancy));
  assign logic_full = (logic_ptrMatch && logic_risingOccupancy);
  assign io_push_ready = (! logic_full);
  assign io_pop_valid = ((! logic_empty) && (! (_zz_2_ && (! logic_full))));
  assign io_pop_payload_data = _zz_3_[7 : 0];
  assign logic_ptrDif = (logic_pushPtr_value - logic_popPtr_value);
  assign io_occupancy = {(logic_risingOccupancy && logic_ptrMatch),logic_ptrDif};
  assign io_availability = {((! logic_risingOccupancy) && logic_ptrMatch),_zz_8_};
  always @ (posedge clk_12M) begin
    if(clockCtrl_systemReset) begin
      logic_pushPtr_value <= (6'b000000);
      logic_popPtr_value <= (6'b000000);
      logic_risingOccupancy <= 1'b0;
      _zz_2_ <= 1'b0;
    end else begin
      logic_pushPtr_value <= logic_pushPtr_valueNext;
      logic_popPtr_value <= logic_popPtr_valueNext;
      _zz_2_ <= (logic_popPtr_valueNext == logic_pushPtr_value);
      if((logic_pushing != logic_popping))begin
        logic_risingOccupancy <= logic_pushing;
      end
      if(io_flush)begin
        logic_risingOccupancy <= 1'b0;
      end
    end
  end

endmodule

module StreamArbiter (
      input   io_inputs_0_valid,
      output  io_inputs_0_ready,
      input   io_inputs_0_payload_last,
      input  [0:0] io_inputs_0_payload_fragment_source,
      input  [0:0] io_inputs_0_payload_fragment_opcode,
      input  [31:0] io_inputs_0_payload_fragment_address,
      input  [4:0] io_inputs_0_payload_fragment_length,
      input  [31:0] io_inputs_0_payload_fragment_data,
      input  [3:0] io_inputs_0_payload_fragment_mask,
      input  [0:0] io_inputs_0_payload_fragment_context,
      input   io_inputs_1_valid,
      output  io_inputs_1_ready,
      input   io_inputs_1_payload_last,
      input  [0:0] io_inputs_1_payload_fragment_source,
      input  [0:0] io_inputs_1_payload_fragment_opcode,
      input  [31:0] io_inputs_1_payload_fragment_address,
      input  [4:0] io_inputs_1_payload_fragment_length,
      input  [31:0] io_inputs_1_payload_fragment_data,
      input  [3:0] io_inputs_1_payload_fragment_mask,
      input  [0:0] io_inputs_1_payload_fragment_context,
      output  io_output_valid,
      input   io_output_ready,
      output  io_output_payload_last,
      output [0:0] io_output_payload_fragment_source,
      output [0:0] io_output_payload_fragment_opcode,
      output [31:0] io_output_payload_fragment_address,
      output [4:0] io_output_payload_fragment_length,
      output [31:0] io_output_payload_fragment_data,
      output [3:0] io_output_payload_fragment_mask,
      output [0:0] io_output_payload_fragment_context,
      output [0:0] io_chosen,
      output [1:0] io_chosenOH,
      input   clk_12M,
      input   clockCtrl_systemReset);
  wire [1:0] _zz_3_;
  wire [1:0] _zz_4_;
  reg  locked;
  wire  maskProposal_0;
  wire  maskProposal_1;
  reg  maskLocked_0;
  reg  maskLocked_1;
  wire  maskRouted_0;
  wire  maskRouted_1;
  wire [1:0] _zz_1_;
  wire  _zz_2_;
  assign _zz_3_ = (_zz_1_ & (~ _zz_4_));
  assign _zz_4_ = (_zz_1_ - (2'b01));
  assign maskRouted_0 = (locked ? maskLocked_0 : maskProposal_0);
  assign maskRouted_1 = (locked ? maskLocked_1 : maskProposal_1);
  assign _zz_1_ = {io_inputs_1_valid,io_inputs_0_valid};
  assign maskProposal_0 = io_inputs_0_valid;
  assign maskProposal_1 = _zz_3_[1];
  assign io_output_valid = ((io_inputs_0_valid && maskRouted_0) || (io_inputs_1_valid && maskRouted_1));
  assign io_output_payload_last = (maskRouted_0 ? io_inputs_0_payload_last : io_inputs_1_payload_last);
  assign io_output_payload_fragment_source = (maskRouted_0 ? io_inputs_0_payload_fragment_source : io_inputs_1_payload_fragment_source);
  assign io_output_payload_fragment_opcode = (maskRouted_0 ? io_inputs_0_payload_fragment_opcode : io_inputs_1_payload_fragment_opcode);
  assign io_output_payload_fragment_address = (maskRouted_0 ? io_inputs_0_payload_fragment_address : io_inputs_1_payload_fragment_address);
  assign io_output_payload_fragment_length = (maskRouted_0 ? io_inputs_0_payload_fragment_length : io_inputs_1_payload_fragment_length);
  assign io_output_payload_fragment_data = (maskRouted_0 ? io_inputs_0_payload_fragment_data : io_inputs_1_payload_fragment_data);
  assign io_output_payload_fragment_mask = (maskRouted_0 ? io_inputs_0_payload_fragment_mask : io_inputs_1_payload_fragment_mask);
  assign io_output_payload_fragment_context = (maskRouted_0 ? io_inputs_0_payload_fragment_context : io_inputs_1_payload_fragment_context);
  assign io_inputs_0_ready = (maskRouted_0 && io_output_ready);
  assign io_inputs_1_ready = (maskRouted_1 && io_output_ready);
  assign io_chosenOH = {maskRouted_1,maskRouted_0};
  assign _zz_2_ = io_chosenOH[1];
  assign io_chosen = _zz_2_;
  always @ (posedge clk_12M) begin
    if(clockCtrl_systemReset) begin
      locked <= 1'b0;
    end else begin
      if(io_output_valid)begin
        locked <= 1'b1;
      end
      if(((io_output_valid && io_output_ready) && io_output_payload_last))begin
        locked <= 1'b0;
      end
    end
  end

  always @ (posedge clk_12M) begin
    if(io_output_valid)begin
      maskLocked_0 <= maskRouted_0;
      maskLocked_1 <= maskRouted_1;
    end
  end

endmodule

module BufferCC_3_ (
      input   io_dataIn,
      output  io_dataOut,
      input   clk_12M);
  reg  buffers_0;
  reg  buffers_1;
  assign io_dataOut = buffers_1;
  always @ (posedge clk_12M) begin
    buffers_0 <= io_dataIn;
    buffers_1 <= buffers_0;
  end

endmodule

module Apb3UartCtrl (
      input  [3:0] io_apb_PADDR,
      input  [0:0] io_apb_PSEL,
      input   io_apb_PENABLE,
      output  io_apb_PREADY,
      input   io_apb_PWRITE,
      input  [31:0] io_apb_PWDATA,
      output reg [31:0] io_apb_PRDATA,
      output  io_uart_txd,
      input   io_uart_rxd,
      output  io_interrupt,
      input   clk_12M,
      input   clockCtrl_systemReset);
  wire  _zz_3_;
  reg  _zz_4_;
  wire  _zz_5_;
  wire  uartCtrl_1__io_write_ready;
  wire  uartCtrl_1__io_read_valid;
  wire [7:0] uartCtrl_1__io_read_payload;
  wire  uartCtrl_1__io_uart_txd;
  wire  bridge_write_streamUnbuffered_queueWithOccupancy_io_push_ready;
  wire  bridge_write_streamUnbuffered_queueWithOccupancy_io_pop_valid;
  wire [7:0] bridge_write_streamUnbuffered_queueWithOccupancy_io_pop_payload;
  wire [0:0] bridge_write_streamUnbuffered_queueWithOccupancy_io_occupancy;
  wire [0:0] bridge_write_streamUnbuffered_queueWithOccupancy_io_availability;
  wire  uartCtrl_1__io_read_queueWithOccupancy_io_push_ready;
  wire  uartCtrl_1__io_read_queueWithOccupancy_io_pop_valid;
  wire [7:0] uartCtrl_1__io_read_queueWithOccupancy_io_pop_payload;
  wire [0:0] uartCtrl_1__io_read_queueWithOccupancy_io_occupancy;
  wire [0:0] uartCtrl_1__io_read_queueWithOccupancy_io_availability;
  wire [0:0] _zz_6_;
  wire [0:0] _zz_7_;
  wire [0:0] _zz_8_;
  wire  busCtrl_askWrite;
  wire  busCtrl_askRead;
  wire  busCtrl_doWrite;
  wire  busCtrl_doRead;
  wire [2:0] bridge_uartConfigReg_frame_dataLength;
  wire `UartStopType_defaultEncoding_type bridge_uartConfigReg_frame_stop;
  wire `UartParityType_defaultEncoding_type bridge_uartConfigReg_frame_parity;
  reg [11:0] bridge_uartConfigReg_clockDivider;
  reg  _zz_1_;
  wire  bridge_write_streamUnbuffered_valid;
  wire  bridge_write_streamUnbuffered_ready;
  wire [7:0] bridge_write_streamUnbuffered_payload;
  reg  bridge_interruptCtrl_writeIntEnable;
  reg  bridge_interruptCtrl_readIntEnable;
  wire  bridge_interruptCtrl_readInt;
  wire  bridge_interruptCtrl_writeInt;
  wire  bridge_interruptCtrl_interrupt;
  wire [7:0] _zz_2_;
  `ifndef SYNTHESIS
  reg [23:0] bridge_uartConfigReg_frame_stop_string;
  reg [31:0] bridge_uartConfigReg_frame_parity_string;
  `endif

  function [11:0] zz_bridge_uartConfigReg_clockDivider(input dummy);
    begin
      zz_bridge_uartConfigReg_clockDivider = (12'b000000000000);
      zz_bridge_uartConfigReg_clockDivider = (12'b000000010011);
    end
  endfunction
  wire [11:0] _zz_9_;
  assign _zz_6_ = io_apb_PWDATA[0 : 0];
  assign _zz_7_ = io_apb_PWDATA[1 : 1];
  assign _zz_8_ = ((1'b1) - bridge_write_streamUnbuffered_queueWithOccupancy_io_occupancy);
  UartCtrl uartCtrl_1_ ( 
    .io_config_frame_dataLength(bridge_uartConfigReg_frame_dataLength),
    .io_config_frame_stop(bridge_uartConfigReg_frame_stop),
    .io_config_frame_parity(bridge_uartConfigReg_frame_parity),
    .io_config_clockDivider(bridge_uartConfigReg_clockDivider),
    .io_write_valid(bridge_write_streamUnbuffered_queueWithOccupancy_io_pop_valid),
    .io_write_ready(uartCtrl_1__io_write_ready),
    .io_write_payload(bridge_write_streamUnbuffered_queueWithOccupancy_io_pop_payload),
    .io_read_valid(uartCtrl_1__io_read_valid),
    .io_read_payload(uartCtrl_1__io_read_payload),
    .io_uart_txd(uartCtrl_1__io_uart_txd),
    .io_uart_rxd(io_uart_rxd),
    .clk_12M(clk_12M),
    .clockCtrl_systemReset(clockCtrl_systemReset) 
  );
  StreamFifo bridge_write_streamUnbuffered_queueWithOccupancy ( 
    .io_push_valid(bridge_write_streamUnbuffered_valid),
    .io_push_ready(bridge_write_streamUnbuffered_queueWithOccupancy_io_push_ready),
    .io_push_payload(bridge_write_streamUnbuffered_payload),
    .io_pop_valid(bridge_write_streamUnbuffered_queueWithOccupancy_io_pop_valid),
    .io_pop_ready(uartCtrl_1__io_write_ready),
    .io_pop_payload(bridge_write_streamUnbuffered_queueWithOccupancy_io_pop_payload),
    .io_flush(_zz_3_),
    .io_occupancy(bridge_write_streamUnbuffered_queueWithOccupancy_io_occupancy),
    .io_availability(bridge_write_streamUnbuffered_queueWithOccupancy_io_availability),
    .clk_12M(clk_12M),
    .clockCtrl_systemReset(clockCtrl_systemReset) 
  );
  StreamFifo uartCtrl_1__io_read_queueWithOccupancy ( 
    .io_push_valid(uartCtrl_1__io_read_valid),
    .io_push_ready(uartCtrl_1__io_read_queueWithOccupancy_io_push_ready),
    .io_push_payload(uartCtrl_1__io_read_payload),
    .io_pop_valid(uartCtrl_1__io_read_queueWithOccupancy_io_pop_valid),
    .io_pop_ready(_zz_4_),
    .io_pop_payload(uartCtrl_1__io_read_queueWithOccupancy_io_pop_payload),
    .io_flush(_zz_5_),
    .io_occupancy(uartCtrl_1__io_read_queueWithOccupancy_io_occupancy),
    .io_availability(uartCtrl_1__io_read_queueWithOccupancy_io_availability),
    .clk_12M(clk_12M),
    .clockCtrl_systemReset(clockCtrl_systemReset) 
  );
  `ifndef SYNTHESIS
  always @(*) begin
    case(bridge_uartConfigReg_frame_stop)
      `UartStopType_defaultEncoding_ONE : bridge_uartConfigReg_frame_stop_string = "ONE";
      `UartStopType_defaultEncoding_TWO : bridge_uartConfigReg_frame_stop_string = "TWO";
      default : bridge_uartConfigReg_frame_stop_string = "???";
    endcase
  end
  always @(*) begin
    case(bridge_uartConfigReg_frame_parity)
      `UartParityType_defaultEncoding_NONE : bridge_uartConfigReg_frame_parity_string = "NONE";
      `UartParityType_defaultEncoding_EVEN : bridge_uartConfigReg_frame_parity_string = "EVEN";
      `UartParityType_defaultEncoding_ODD : bridge_uartConfigReg_frame_parity_string = "ODD ";
      default : bridge_uartConfigReg_frame_parity_string = "????";
    endcase
  end
  `endif

  assign io_uart_txd = uartCtrl_1__io_uart_txd;
  assign io_apb_PREADY = 1'b1;
  always @ (*) begin
    io_apb_PRDATA = (32'b00000000000000000000000000000000);
    case(io_apb_PADDR)
      4'b0000 : begin
        io_apb_PRDATA[16 : 16] = (uartCtrl_1__io_read_queueWithOccupancy_io_pop_valid ^ 1'b0);
        io_apb_PRDATA[7 : 0] = uartCtrl_1__io_read_queueWithOccupancy_io_pop_payload;
      end
      4'b0100 : begin
        io_apb_PRDATA[16 : 16] = _zz_8_;
        io_apb_PRDATA[24 : 24] = uartCtrl_1__io_read_queueWithOccupancy_io_occupancy;
        io_apb_PRDATA[0 : 0] = bridge_interruptCtrl_writeIntEnable;
        io_apb_PRDATA[1 : 1] = bridge_interruptCtrl_readIntEnable;
        io_apb_PRDATA[8 : 8] = bridge_interruptCtrl_writeInt;
        io_apb_PRDATA[9 : 9] = bridge_interruptCtrl_readInt;
      end
      default : begin
      end
    endcase
  end

  assign busCtrl_askWrite = ((io_apb_PSEL[0] && io_apb_PENABLE) && io_apb_PWRITE);
  assign busCtrl_askRead = ((io_apb_PSEL[0] && io_apb_PENABLE) && (! io_apb_PWRITE));
  assign busCtrl_doWrite = (((io_apb_PSEL[0] && io_apb_PENABLE) && io_apb_PREADY) && io_apb_PWRITE);
  assign busCtrl_doRead = (((io_apb_PSEL[0] && io_apb_PENABLE) && io_apb_PREADY) && (! io_apb_PWRITE));
  assign _zz_9_ = zz_bridge_uartConfigReg_clockDivider(1'b0);
  always @ (*) bridge_uartConfigReg_clockDivider = _zz_9_;
  assign bridge_uartConfigReg_frame_dataLength = (3'b111);
  assign bridge_uartConfigReg_frame_parity = `UartParityType_defaultEncoding_NONE;
  assign bridge_uartConfigReg_frame_stop = `UartStopType_defaultEncoding_ONE;
  always @ (*) begin
    _zz_1_ = 1'b0;
    case(io_apb_PADDR)
      4'b0000 : begin
        if(busCtrl_doWrite)begin
          _zz_1_ = 1'b1;
        end
      end
      4'b0100 : begin
      end
      default : begin
      end
    endcase
  end

  assign bridge_write_streamUnbuffered_valid = _zz_1_;
  assign bridge_write_streamUnbuffered_payload = _zz_2_;
  assign bridge_write_streamUnbuffered_ready = bridge_write_streamUnbuffered_queueWithOccupancy_io_push_ready;
  always @ (*) begin
    _zz_4_ = 1'b0;
    case(io_apb_PADDR)
      4'b0000 : begin
        if(busCtrl_doRead)begin
          _zz_4_ = 1'b1;
        end
      end
      4'b0100 : begin
      end
      default : begin
      end
    endcase
  end

  assign bridge_interruptCtrl_readInt = (bridge_interruptCtrl_readIntEnable && uartCtrl_1__io_read_queueWithOccupancy_io_pop_valid);
  assign bridge_interruptCtrl_writeInt = (bridge_interruptCtrl_writeIntEnable && (! bridge_write_streamUnbuffered_queueWithOccupancy_io_pop_valid));
  assign bridge_interruptCtrl_interrupt = (bridge_interruptCtrl_readInt || bridge_interruptCtrl_writeInt);
  assign io_interrupt = bridge_interruptCtrl_interrupt;
  assign _zz_2_ = io_apb_PWDATA[7 : 0];
  assign _zz_3_ = 1'b0;
  assign _zz_5_ = 1'b0;
  always @ (posedge clk_12M) begin
    if(clockCtrl_systemReset) begin
      bridge_interruptCtrl_writeIntEnable <= 1'b0;
      bridge_interruptCtrl_readIntEnable <= 1'b0;
    end else begin
      case(io_apb_PADDR)
        4'b0000 : begin
        end
        4'b0100 : begin
          if(busCtrl_doWrite)begin
            bridge_interruptCtrl_writeIntEnable <= _zz_6_[0];
            bridge_interruptCtrl_readIntEnable <= _zz_7_[0];
          end
        end
        default : begin
        end
      endcase
    end
  end

endmodule

module Apb3Gpio2 (
      input  [3:0] io_gpio_read,
      output reg [3:0] io_gpio_write,
      output reg [3:0] io_gpio_writeEnable,
      input  [11:0] io_bus_PADDR,
      input  [0:0] io_bus_PSEL,
      input   io_bus_PENABLE,
      output  io_bus_PREADY,
      input   io_bus_PWRITE,
      input  [31:0] io_bus_PWDATA,
      output reg [31:0] io_bus_PRDATA,
      output  io_bus_PSLVERROR,
      output reg [3:0] io_interrupt,
      input   clk_12M,
      input   clockCtrl_systemReset);
  wire [3:0] bufferCC_4__io_dataOut;
  wire [0:0] _zz_17_;
  wire [0:0] _zz_18_;
  wire [0:0] _zz_19_;
  wire [0:0] _zz_20_;
  wire [0:0] _zz_21_;
  wire [0:0] _zz_22_;
  wire [0:0] _zz_23_;
  wire [0:0] _zz_24_;
  wire [0:0] _zz_25_;
  wire [0:0] _zz_26_;
  wire [0:0] _zz_27_;
  wire [0:0] _zz_28_;
  wire [0:0] _zz_29_;
  wire [0:0] _zz_30_;
  wire [0:0] _zz_31_;
  wire [0:0] _zz_32_;
  wire  mapper_askWrite;
  wire  mapper_askRead;
  wire  mapper_doWrite;
  wire  mapper_doRead;
  wire [3:0] syncronized;
  reg [3:0] last;
  reg  _zz_1_;
  reg  _zz_2_;
  reg  _zz_3_;
  reg  _zz_4_;
  reg  _zz_5_;
  reg  _zz_6_;
  reg  _zz_7_;
  reg  _zz_8_;
  reg [3:0] interrupt_enable_high;
  reg [3:0] interrupt_enable_low;
  reg [3:0] interrupt_enable_rise;
  reg [3:0] interrupt_enable_fall;
  wire [3:0] interrupt_valid;
  reg  _zz_9_;
  reg  _zz_10_;
  reg  _zz_11_;
  reg  _zz_12_;
  reg  _zz_13_;
  reg  _zz_14_;
  reg  _zz_15_;
  reg  _zz_16_;
  assign _zz_17_ = io_bus_PWDATA[0 : 0];
  assign _zz_18_ = io_bus_PWDATA[1 : 1];
  assign _zz_19_ = io_bus_PWDATA[2 : 2];
  assign _zz_20_ = io_bus_PWDATA[3 : 3];
  assign _zz_21_ = io_bus_PWDATA[0 : 0];
  assign _zz_22_ = io_bus_PWDATA[1 : 1];
  assign _zz_23_ = io_bus_PWDATA[2 : 2];
  assign _zz_24_ = io_bus_PWDATA[3 : 3];
  assign _zz_25_ = io_bus_PWDATA[0 : 0];
  assign _zz_26_ = io_bus_PWDATA[1 : 1];
  assign _zz_27_ = io_bus_PWDATA[0 : 0];
  assign _zz_28_ = io_bus_PWDATA[1 : 1];
  assign _zz_29_ = io_bus_PWDATA[0 : 0];
  assign _zz_30_ = io_bus_PWDATA[1 : 1];
  assign _zz_31_ = io_bus_PWDATA[0 : 0];
  assign _zz_32_ = io_bus_PWDATA[1 : 1];
  BufferCC_2_ bufferCC_4_ ( 
    .io_dataIn(io_gpio_read),
    .io_dataOut(bufferCC_4__io_dataOut),
    .clk_12M(clk_12M),
    .clockCtrl_systemReset(clockCtrl_systemReset) 
  );
  assign io_bus_PREADY = 1'b1;
  always @ (*) begin
    io_bus_PRDATA = (32'b00000000000000000000000000000000);
    case(io_bus_PADDR)
      12'b000000000000 : begin
        io_bus_PRDATA[0 : 0] = syncronized[0];
        io_bus_PRDATA[1 : 1] = syncronized[1];
        io_bus_PRDATA[2 : 2] = syncronized[2];
        io_bus_PRDATA[3 : 3] = syncronized[3];
      end
      12'b000000000100 : begin
        io_bus_PRDATA[0 : 0] = _zz_1_;
        io_bus_PRDATA[1 : 1] = _zz_3_;
        io_bus_PRDATA[2 : 2] = _zz_5_;
        io_bus_PRDATA[3 : 3] = _zz_7_;
      end
      12'b000000001000 : begin
        io_bus_PRDATA[0 : 0] = _zz_2_;
        io_bus_PRDATA[1 : 1] = _zz_4_;
        io_bus_PRDATA[2 : 2] = _zz_6_;
        io_bus_PRDATA[3 : 3] = _zz_8_;
      end
      12'b000000100000 : begin
        io_bus_PRDATA[0 : 0] = _zz_9_;
        io_bus_PRDATA[1 : 1] = _zz_13_;
      end
      12'b000000100100 : begin
        io_bus_PRDATA[0 : 0] = _zz_10_;
        io_bus_PRDATA[1 : 1] = _zz_14_;
      end
      12'b000000101000 : begin
        io_bus_PRDATA[0 : 0] = _zz_11_;
        io_bus_PRDATA[1 : 1] = _zz_15_;
      end
      12'b000000101100 : begin
        io_bus_PRDATA[0 : 0] = _zz_12_;
        io_bus_PRDATA[1 : 1] = _zz_16_;
      end
      default : begin
      end
    endcase
  end

  assign io_bus_PSLVERROR = 1'b0;
  assign mapper_askWrite = ((io_bus_PSEL[0] && io_bus_PENABLE) && io_bus_PWRITE);
  assign mapper_askRead = ((io_bus_PSEL[0] && io_bus_PENABLE) && (! io_bus_PWRITE));
  assign mapper_doWrite = (((io_bus_PSEL[0] && io_bus_PENABLE) && io_bus_PREADY) && io_bus_PWRITE);
  assign mapper_doRead = (((io_bus_PSEL[0] && io_bus_PENABLE) && io_bus_PREADY) && (! io_bus_PWRITE));
  assign syncronized = bufferCC_4__io_dataOut;
  always @ (*) begin
    io_gpio_write[0] = _zz_1_;
    io_gpio_write[1] = _zz_3_;
    io_gpio_write[2] = _zz_5_;
    io_gpio_write[3] = _zz_7_;
  end

  always @ (*) begin
    io_gpio_writeEnable[0] = _zz_2_;
    io_gpio_writeEnable[1] = _zz_4_;
    io_gpio_writeEnable[2] = _zz_6_;
    io_gpio_writeEnable[3] = _zz_8_;
  end

  assign interrupt_valid = ((((interrupt_enable_high & syncronized) | (interrupt_enable_low & (~ syncronized))) | (interrupt_enable_rise & (syncronized & (~ last)))) | (interrupt_enable_fall & ((~ syncronized) & last)));
  always @ (*) begin
    io_interrupt[0] = interrupt_valid[0];
    io_interrupt[1] = interrupt_valid[1];
    io_interrupt[2] = 1'b0;
    io_interrupt[3] = 1'b0;
  end

  always @ (*) begin
    interrupt_enable_rise[0] = _zz_9_;
    interrupt_enable_rise[1] = _zz_13_;
    interrupt_enable_rise[2] = 1'b0;
    interrupt_enable_rise[3] = 1'b0;
  end

  always @ (*) begin
    interrupt_enable_fall[0] = _zz_10_;
    interrupt_enable_fall[1] = _zz_14_;
    interrupt_enable_fall[2] = 1'b0;
    interrupt_enable_fall[3] = 1'b0;
  end

  always @ (*) begin
    interrupt_enable_high[0] = _zz_11_;
    interrupt_enable_high[1] = _zz_15_;
    interrupt_enable_high[2] = 1'b0;
    interrupt_enable_high[3] = 1'b0;
  end

  always @ (*) begin
    interrupt_enable_low[0] = _zz_12_;
    interrupt_enable_low[1] = _zz_16_;
    interrupt_enable_low[2] = 1'b0;
    interrupt_enable_low[3] = 1'b0;
  end

  always @ (posedge clk_12M) begin
    last <= syncronized;
    case(io_bus_PADDR)
      12'b000000000000 : begin
      end
      12'b000000000100 : begin
        if(mapper_doWrite)begin
          _zz_1_ <= _zz_17_[0];
          _zz_3_ <= _zz_18_[0];
          _zz_5_ <= _zz_19_[0];
          _zz_7_ <= _zz_20_[0];
        end
      end
      12'b000000001000 : begin
      end
      12'b000000100000 : begin
      end
      12'b000000100100 : begin
      end
      12'b000000101000 : begin
      end
      12'b000000101100 : begin
      end
      default : begin
      end
    endcase
  end

  always @ (posedge clk_12M) begin
    if(clockCtrl_systemReset) begin
      _zz_2_ <= 1'b0;
      _zz_4_ <= 1'b0;
      _zz_6_ <= 1'b0;
      _zz_8_ <= 1'b0;
      _zz_9_ <= 1'b0;
      _zz_10_ <= 1'b0;
      _zz_11_ <= 1'b0;
      _zz_12_ <= 1'b0;
      _zz_13_ <= 1'b0;
      _zz_14_ <= 1'b0;
      _zz_15_ <= 1'b0;
      _zz_16_ <= 1'b0;
    end else begin
      case(io_bus_PADDR)
        12'b000000000000 : begin
        end
        12'b000000000100 : begin
        end
        12'b000000001000 : begin
          if(mapper_doWrite)begin
            _zz_2_ <= _zz_21_[0];
            _zz_4_ <= _zz_22_[0];
            _zz_6_ <= _zz_23_[0];
            _zz_8_ <= _zz_24_[0];
          end
        end
        12'b000000100000 : begin
          if(mapper_doWrite)begin
            _zz_9_ <= _zz_25_[0];
            _zz_13_ <= _zz_26_[0];
          end
        end
        12'b000000100100 : begin
          if(mapper_doWrite)begin
            _zz_10_ <= _zz_27_[0];
            _zz_14_ <= _zz_28_[0];
          end
        end
        12'b000000101000 : begin
          if(mapper_doWrite)begin
            _zz_11_ <= _zz_29_[0];
            _zz_15_ <= _zz_30_[0];
          end
        end
        12'b000000101100 : begin
          if(mapper_doWrite)begin
            _zz_12_ <= _zz_31_[0];
            _zz_16_ <= _zz_32_[0];
          end
        end
        default : begin
        end
      endcase
    end
  end

endmodule

module MachineTimer (
      input  [3:0] io_bus_PADDR,
      input  [0:0] io_bus_PSEL,
      input   io_bus_PENABLE,
      output  io_bus_PREADY,
      input   io_bus_PWRITE,
      input  [31:0] io_bus_PWDATA,
      output reg [31:0] io_bus_PRDATA,
      output  io_bus_PSLVERROR,
      output  io_mTimeInterrupt,
      input   clk_12M,
      input   clockCtrl_systemReset);
  wire [63:0] _zz_2_;
  wire [31:0] _zz_3_;
  wire [31:0] _zz_4_;
  wire [31:0] _zz_5_;
  wire [31:0] _zz_6_;
  wire  mapper_askWrite;
  wire  mapper_askRead;
  wire  mapper_doWrite;
  wire  mapper_doRead;
  reg [63:0] counter;
  reg [63:0] cmp;
  reg  interrupt;
  wire [63:0] _zz_1_;
  assign _zz_2_ = (counter - cmp);
  assign _zz_3_ = io_bus_PWDATA[31 : 0];
  assign _zz_4_ = _zz_3_;
  assign _zz_5_ = io_bus_PWDATA[31 : 0];
  assign _zz_6_ = _zz_5_;
  assign io_bus_PREADY = 1'b1;
  always @ (*) begin
    io_bus_PRDATA = (32'bxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx);
    case(io_bus_PADDR)
      4'b0000 : begin
        io_bus_PRDATA[31 : 0] = _zz_1_[31 : 0];
      end
      4'b0100 : begin
        io_bus_PRDATA[31 : 0] = _zz_1_[63 : 32];
      end
      4'b1000 : begin
      end
      4'b1100 : begin
      end
      default : begin
      end
    endcase
  end

  assign io_bus_PSLVERROR = 1'b0;
  assign mapper_askWrite = ((io_bus_PSEL[0] && io_bus_PENABLE) && io_bus_PWRITE);
  assign mapper_askRead = ((io_bus_PSEL[0] && io_bus_PENABLE) && (! io_bus_PWRITE));
  assign mapper_doWrite = (((io_bus_PSEL[0] && io_bus_PENABLE) && io_bus_PREADY) && io_bus_PWRITE);
  assign mapper_doRead = (((io_bus_PSEL[0] && io_bus_PENABLE) && io_bus_PREADY) && (! io_bus_PWRITE));
  assign io_mTimeInterrupt = interrupt;
  assign _zz_1_ = counter;
  always @ (posedge clk_12M) begin
    if(clockCtrl_systemReset) begin
      counter <= (64'b0000000000000000000000000000000000000000000000000000000000000000);
    end else begin
      counter <= (counter + (64'b0000000000000000000000000000000000000000000000000000000000000001));
    end
  end

  always @ (posedge clk_12M) begin
    interrupt <= (! _zz_2_[63]);
    case(io_bus_PADDR)
      4'b0000 : begin
      end
      4'b0100 : begin
      end
      4'b1000 : begin
        if(mapper_doWrite)begin
          cmp[31 : 0] <= _zz_4_;
        end
      end
      4'b1100 : begin
        if(mapper_doWrite)begin
          cmp[63 : 32] <= _zz_6_;
        end
      end
      default : begin
      end
    endcase
  end

endmodule

module VexRiscv (
      input   timerInterrupt,
      input   externalInterrupt,
      input   softwareInterrupt,
      input   debug_bus_cmd_valid,
      output reg  debug_bus_cmd_ready,
      input   debug_bus_cmd_payload_wr,
      input  [7:0] debug_bus_cmd_payload_address,
      input  [31:0] debug_bus_cmd_payload_data,
      output reg [31:0] debug_bus_rsp_data,
      output  debug_resetOut,
      output  iBus_cmd_valid,
      input   iBus_cmd_ready,
      output reg [31:0] iBus_cmd_payload_address,
      output [2:0] iBus_cmd_payload_size,
      input   iBus_rsp_valid,
      input  [31:0] iBus_rsp_payload_data,
      input   iBus_rsp_payload_error,
      output  dBus_cmd_valid,
      input   dBus_cmd_ready,
      output  dBus_cmd_payload_wr,
      output [31:0] dBus_cmd_payload_address,
      output [31:0] dBus_cmd_payload_data,
      output [1:0] dBus_cmd_payload_size,
      input   dBus_rsp_ready,
      input   dBus_rsp_error,
      input  [31:0] dBus_rsp_data,
      input   clk_12M,
      input   clockCtrl_systemReset,
      input   clockCtrl_resetUnbuffered_regNext);
  wire  _zz_143_;
  wire  _zz_144_;
  wire  _zz_145_;
  wire  _zz_146_;
  wire [31:0] _zz_147_;
  wire  _zz_148_;
  wire  _zz_149_;
  wire  _zz_150_;
  wire  _zz_151_;
  wire  _zz_152_;
  wire  _zz_153_;
  wire  _zz_154_;
  wire  _zz_155_;
  wire  _zz_156_;
  wire  _zz_157_;
  wire [31:0] _zz_158_;
  reg  _zz_159_;
  reg [31:0] _zz_160_;
  reg [31:0] _zz_161_;
  reg [31:0] _zz_162_;
  wire  IBusCachedPlugin_cache_io_cpu_prefetch_haltIt;
  wire  IBusCachedPlugin_cache_io_cpu_fetch_error;
  wire  IBusCachedPlugin_cache_io_cpu_fetch_mmuRefilling;
  wire  IBusCachedPlugin_cache_io_cpu_fetch_mmuException;
  wire [31:0] IBusCachedPlugin_cache_io_cpu_fetch_data;
  wire  IBusCachedPlugin_cache_io_cpu_fetch_cacheMiss;
  wire [31:0] IBusCachedPlugin_cache_io_cpu_fetch_physicalAddress;
  wire  IBusCachedPlugin_cache_io_cpu_fetch_haltIt;
  wire  IBusCachedPlugin_cache_io_cpu_fetch_mmuBus_cmd_isValid;
  wire [31:0] IBusCachedPlugin_cache_io_cpu_fetch_mmuBus_cmd_virtualAddress;
  wire  IBusCachedPlugin_cache_io_cpu_fetch_mmuBus_cmd_bypassTranslation;
  wire  IBusCachedPlugin_cache_io_cpu_fetch_mmuBus_end;
  wire [31:0] IBusCachedPlugin_cache_io_cpu_decode_data;
  wire [31:0] IBusCachedPlugin_cache_io_cpu_decode_physicalAddress;
  wire  IBusCachedPlugin_cache_io_mem_cmd_valid;
  wire [31:0] IBusCachedPlugin_cache_io_mem_cmd_payload_address;
  wire [2:0] IBusCachedPlugin_cache_io_mem_cmd_payload_size;
  wire  _zz_163_;
  wire  _zz_164_;
  wire  _zz_165_;
  wire  _zz_166_;
  wire  _zz_167_;
  wire  _zz_168_;
  wire  _zz_169_;
  wire  _zz_170_;
  wire  _zz_171_;
  wire  _zz_172_;
  wire [1:0] _zz_173_;
  wire  _zz_174_;
  wire  _zz_175_;
  wire  _zz_176_;
  wire  _zz_177_;
  wire  _zz_178_;
  wire  _zz_179_;
  wire  _zz_180_;
  wire  _zz_181_;
  wire [5:0] _zz_182_;
  wire  _zz_183_;
  wire  _zz_184_;
  wire  _zz_185_;
  wire  _zz_186_;
  wire [1:0] _zz_187_;
  wire  _zz_188_;
  wire [2:0] _zz_189_;
  wire [2:0] _zz_190_;
  wire [31:0] _zz_191_;
  wire [0:0] _zz_192_;
  wire [0:0] _zz_193_;
  wire [0:0] _zz_194_;
  wire [0:0] _zz_195_;
  wire [0:0] _zz_196_;
  wire [0:0] _zz_197_;
  wire [0:0] _zz_198_;
  wire [0:0] _zz_199_;
  wire [0:0] _zz_200_;
  wire [0:0] _zz_201_;
  wire [0:0] _zz_202_;
  wire [0:0] _zz_203_;
  wire [0:0] _zz_204_;
  wire [0:0] _zz_205_;
  wire [2:0] _zz_206_;
  wire [4:0] _zz_207_;
  wire [11:0] _zz_208_;
  wire [11:0] _zz_209_;
  wire [31:0] _zz_210_;
  wire [31:0] _zz_211_;
  wire [31:0] _zz_212_;
  wire [31:0] _zz_213_;
  wire [31:0] _zz_214_;
  wire [31:0] _zz_215_;
  wire [31:0] _zz_216_;
  wire [31:0] _zz_217_;
  wire [32:0] _zz_218_;
  wire [19:0] _zz_219_;
  wire [11:0] _zz_220_;
  wire [11:0] _zz_221_;
  wire [0:0] _zz_222_;
  wire [0:0] _zz_223_;
  wire [0:0] _zz_224_;
  wire [0:0] _zz_225_;
  wire [30:0] _zz_226_;
  wire [30:0] _zz_227_;
  wire [30:0] _zz_228_;
  wire [30:0] _zz_229_;
  wire [0:0] _zz_230_;
  wire [0:0] _zz_231_;
  wire [0:0] _zz_232_;
  wire [0:0] _zz_233_;
  wire [0:0] _zz_234_;
  wire [0:0] _zz_235_;
  wire  _zz_236_;
  wire  _zz_237_;
  wire [1:0] _zz_238_;
  wire [31:0] _zz_239_;
  wire [31:0] _zz_240_;
  wire [31:0] _zz_241_;
  wire [31:0] _zz_242_;
  wire  _zz_243_;
  wire  _zz_244_;
  wire  _zz_245_;
  wire [1:0] _zz_246_;
  wire [1:0] _zz_247_;
  wire  _zz_248_;
  wire [0:0] _zz_249_;
  wire [21:0] _zz_250_;
  wire [31:0] _zz_251_;
  wire [31:0] _zz_252_;
  wire [31:0] _zz_253_;
  wire [31:0] _zz_254_;
  wire [31:0] _zz_255_;
  wire [31:0] _zz_256_;
  wire  _zz_257_;
  wire [1:0] _zz_258_;
  wire [1:0] _zz_259_;
  wire  _zz_260_;
  wire [0:0] _zz_261_;
  wire [18:0] _zz_262_;
  wire [31:0] _zz_263_;
  wire [31:0] _zz_264_;
  wire [31:0] _zz_265_;
  wire [31:0] _zz_266_;
  wire [31:0] _zz_267_;
  wire [31:0] _zz_268_;
  wire  _zz_269_;
  wire [0:0] _zz_270_;
  wire [0:0] _zz_271_;
  wire  _zz_272_;
  wire [0:0] _zz_273_;
  wire [15:0] _zz_274_;
  wire [31:0] _zz_275_;
  wire [31:0] _zz_276_;
  wire  _zz_277_;
  wire [5:0] _zz_278_;
  wire [5:0] _zz_279_;
  wire  _zz_280_;
  wire [0:0] _zz_281_;
  wire [11:0] _zz_282_;
  wire  _zz_283_;
  wire [0:0] _zz_284_;
  wire [2:0] _zz_285_;
  wire  _zz_286_;
  wire  _zz_287_;
  wire [2:0] _zz_288_;
  wire [2:0] _zz_289_;
  wire  _zz_290_;
  wire [0:0] _zz_291_;
  wire [8:0] _zz_292_;
  wire [31:0] _zz_293_;
  wire [31:0] _zz_294_;
  wire [31:0] _zz_295_;
  wire [0:0] _zz_296_;
  wire [0:0] _zz_297_;
  wire [31:0] _zz_298_;
  wire [31:0] _zz_299_;
  wire  _zz_300_;
  wire [0:0] _zz_301_;
  wire [0:0] _zz_302_;
  wire [0:0] _zz_303_;
  wire [0:0] _zz_304_;
  wire [0:0] _zz_305_;
  wire [0:0] _zz_306_;
  wire  _zz_307_;
  wire [0:0] _zz_308_;
  wire [6:0] _zz_309_;
  wire [31:0] _zz_310_;
  wire [31:0] _zz_311_;
  wire [31:0] _zz_312_;
  wire [31:0] _zz_313_;
  wire [31:0] _zz_314_;
  wire [31:0] _zz_315_;
  wire [31:0] _zz_316_;
  wire [31:0] _zz_317_;
  wire [31:0] _zz_318_;
  wire [31:0] _zz_319_;
  wire [31:0] _zz_320_;
  wire [31:0] _zz_321_;
  wire [31:0] _zz_322_;
  wire [0:0] _zz_323_;
  wire [0:0] _zz_324_;
  wire [0:0] _zz_325_;
  wire [0:0] _zz_326_;
  wire  _zz_327_;
  wire [0:0] _zz_328_;
  wire [4:0] _zz_329_;
  wire [31:0] _zz_330_;
  wire [31:0] _zz_331_;
  wire [31:0] _zz_332_;
  wire  _zz_333_;
  wire [0:0] _zz_334_;
  wire [0:0] _zz_335_;
  wire [0:0] _zz_336_;
  wire [0:0] _zz_337_;
  wire  _zz_338_;
  wire [0:0] _zz_339_;
  wire [1:0] _zz_340_;
  wire [31:0] _zz_341_;
  wire [31:0] _zz_342_;
  wire [31:0] _zz_343_;
  wire [0:0] _zz_344_;
  wire [0:0] _zz_345_;
  wire [0:0] _zz_346_;
  wire [2:0] _zz_347_;
  wire `AluBitwiseCtrlEnum_defaultEncoding_type decode_ALU_BITWISE_CTRL;
  wire `AluBitwiseCtrlEnum_defaultEncoding_type _zz_1_;
  wire `AluBitwiseCtrlEnum_defaultEncoding_type _zz_2_;
  wire `AluBitwiseCtrlEnum_defaultEncoding_type _zz_3_;
  wire [1:0] memory_MEMORY_ADDRESS_LOW;
  wire [1:0] execute_MEMORY_ADDRESS_LOW;
  wire  decode_MEMORY_STORE;
  wire  decode_BYPASSABLE_EXECUTE_STAGE;
  wire [31:0] decode_SRC2;
  wire  execute_BYPASSABLE_MEMORY_STAGE;
  wire  decode_BYPASSABLE_MEMORY_STAGE;
  wire [31:0] memory_PC;
  wire  decode_CSR_WRITE_OPCODE;
  wire `EnvCtrlEnum_defaultEncoding_type _zz_4_;
  wire `EnvCtrlEnum_defaultEncoding_type _zz_5_;
  wire `EnvCtrlEnum_defaultEncoding_type _zz_6_;
  wire `EnvCtrlEnum_defaultEncoding_type _zz_7_;
  wire `EnvCtrlEnum_defaultEncoding_type decode_ENV_CTRL;
  wire `EnvCtrlEnum_defaultEncoding_type _zz_8_;
  wire `EnvCtrlEnum_defaultEncoding_type _zz_9_;
  wire `EnvCtrlEnum_defaultEncoding_type _zz_10_;
  wire `ShiftCtrlEnum_defaultEncoding_type decode_SHIFT_CTRL;
  wire `ShiftCtrlEnum_defaultEncoding_type _zz_11_;
  wire `ShiftCtrlEnum_defaultEncoding_type _zz_12_;
  wire `ShiftCtrlEnum_defaultEncoding_type _zz_13_;
  wire `AluCtrlEnum_defaultEncoding_type decode_ALU_CTRL;
  wire `AluCtrlEnum_defaultEncoding_type _zz_14_;
  wire `AluCtrlEnum_defaultEncoding_type _zz_15_;
  wire `AluCtrlEnum_defaultEncoding_type _zz_16_;
  wire `BranchCtrlEnum_defaultEncoding_type decode_BRANCH_CTRL;
  wire `BranchCtrlEnum_defaultEncoding_type _zz_17_;
  wire `BranchCtrlEnum_defaultEncoding_type _zz_18_;
  wire `BranchCtrlEnum_defaultEncoding_type _zz_19_;
  wire [31:0] writeBack_FORMAL_PC_NEXT;
  wire [31:0] memory_FORMAL_PC_NEXT;
  wire [31:0] execute_FORMAL_PC_NEXT;
  wire [31:0] decode_FORMAL_PC_NEXT;
  wire [31:0] writeBack_REGFILE_WRITE_DATA;
  wire [31:0] execute_REGFILE_WRITE_DATA;
  wire [31:0] memory_MEMORY_READ_DATA;
  wire [31:0] decode_RS1;
  wire  decode_DO_EBREAK;
  wire [31:0] decode_RS2;
  wire  execute_BRANCH_DO;
  wire [31:0] decode_SRC1;
  wire  decode_IS_CSR;
  wire  decode_SRC2_FORCE_ZERO;
  wire  decode_SRC_LESS_UNSIGNED;
  wire  decode_CSR_READ_OPCODE;
  wire  decode_MEMORY_ENABLE;
  wire [31:0] execute_BRANCH_CALC;
  wire  execute_DO_EBREAK;
  wire  decode_IS_EBREAK;
  wire  _zz_20_;
  wire  execute_CSR_READ_OPCODE;
  wire  execute_CSR_WRITE_OPCODE;
  wire  execute_IS_CSR;
  wire `EnvCtrlEnum_defaultEncoding_type memory_ENV_CTRL;
  wire `EnvCtrlEnum_defaultEncoding_type _zz_21_;
  wire `EnvCtrlEnum_defaultEncoding_type execute_ENV_CTRL;
  wire `EnvCtrlEnum_defaultEncoding_type _zz_22_;
  wire  _zz_23_;
  wire  _zz_24_;
  wire `EnvCtrlEnum_defaultEncoding_type writeBack_ENV_CTRL;
  wire `EnvCtrlEnum_defaultEncoding_type _zz_25_;
  wire  decode_RS2_USE;
  wire  decode_RS1_USE;
  wire  execute_REGFILE_WRITE_VALID;
  wire  execute_BYPASSABLE_EXECUTE_STAGE;
  wire  memory_REGFILE_WRITE_VALID;
  wire [31:0] memory_INSTRUCTION;
  wire  memory_BYPASSABLE_MEMORY_STAGE;
  wire  writeBack_REGFILE_WRITE_VALID;
  wire [31:0] memory_BRANCH_CALC;
  wire  memory_BRANCH_DO;
  wire [31:0] _zz_26_;
  wire [31:0] execute_PC;
  wire [31:0] execute_RS1;
  wire `BranchCtrlEnum_defaultEncoding_type execute_BRANCH_CTRL;
  wire `BranchCtrlEnum_defaultEncoding_type _zz_27_;
  wire  _zz_28_;
  reg [31:0] _zz_29_;
  wire [31:0] memory_REGFILE_WRITE_DATA;
  wire `ShiftCtrlEnum_defaultEncoding_type execute_SHIFT_CTRL;
  wire `ShiftCtrlEnum_defaultEncoding_type _zz_30_;
  wire  _zz_31_;
  wire [31:0] _zz_32_;
  wire [31:0] _zz_33_;
  wire  execute_SRC_LESS_UNSIGNED;
  wire  execute_SRC2_FORCE_ZERO;
  wire  execute_SRC_USE_SUB_LESS;
  wire [31:0] _zz_34_;
  wire [31:0] _zz_35_;
  wire `Src2CtrlEnum_defaultEncoding_type decode_SRC2_CTRL;
  wire `Src2CtrlEnum_defaultEncoding_type _zz_36_;
  wire [31:0] _zz_37_;
  wire [31:0] _zz_38_;
  wire `Src1CtrlEnum_defaultEncoding_type decode_SRC1_CTRL;
  wire `Src1CtrlEnum_defaultEncoding_type _zz_39_;
  wire [31:0] _zz_40_;
  wire  decode_SRC_USE_SUB_LESS;
  wire  decode_SRC_ADD_ZERO;
  wire  _zz_41_;
  wire [31:0] execute_SRC_ADD_SUB;
  wire  execute_SRC_LESS;
  wire `AluCtrlEnum_defaultEncoding_type execute_ALU_CTRL;
  wire `AluCtrlEnum_defaultEncoding_type _zz_42_;
  wire [31:0] _zz_43_;
  wire [31:0] execute_SRC2;
  wire [31:0] execute_SRC1;
  wire `AluBitwiseCtrlEnum_defaultEncoding_type execute_ALU_BITWISE_CTRL;
  wire `AluBitwiseCtrlEnum_defaultEncoding_type _zz_44_;
  wire [31:0] _zz_45_;
  wire  _zz_46_;
  reg  _zz_47_;
  wire [31:0] _zz_48_;
  wire [31:0] _zz_49_;
  wire [31:0] decode_INSTRUCTION_ANTICIPATED;
  reg  decode_REGFILE_WRITE_VALID;
  wire  _zz_50_;
  wire  _zz_51_;
  wire `ShiftCtrlEnum_defaultEncoding_type _zz_52_;
  wire  _zz_53_;
  wire  _zz_54_;
  wire  _zz_55_;
  wire  _zz_56_;
  wire `AluBitwiseCtrlEnum_defaultEncoding_type _zz_57_;
  wire `Src1CtrlEnum_defaultEncoding_type _zz_58_;
  wire  _zz_59_;
  wire  _zz_60_;
  wire `AluCtrlEnum_defaultEncoding_type _zz_61_;
  wire  _zz_62_;
  wire `BranchCtrlEnum_defaultEncoding_type _zz_63_;
  wire  _zz_64_;
  wire `Src2CtrlEnum_defaultEncoding_type _zz_65_;
  wire `EnvCtrlEnum_defaultEncoding_type _zz_66_;
  wire  _zz_67_;
  wire  _zz_68_;
  wire  _zz_69_;
  wire  writeBack_MEMORY_STORE;
  reg [31:0] _zz_70_;
  wire  writeBack_MEMORY_ENABLE;
  wire [1:0] writeBack_MEMORY_ADDRESS_LOW;
  wire [31:0] writeBack_MEMORY_READ_DATA;
  wire  memory_MEMORY_STORE;
  wire  memory_MEMORY_ENABLE;
  wire [31:0] _zz_71_;
  wire [31:0] execute_SRC_ADD;
  wire [1:0] _zz_72_;
  wire [31:0] execute_RS2;
  wire [31:0] execute_INSTRUCTION;
  wire  execute_MEMORY_STORE;
  wire  execute_MEMORY_ENABLE;
  wire  execute_ALIGNEMENT_FAULT;
  wire  decode_FLUSH_ALL;
  reg  IBusCachedPlugin_rsp_issueDetected;
  reg  _zz_73_;
  reg [31:0] _zz_74_;
  reg [31:0] _zz_75_;
  wire [31:0] decode_PC;
  wire [31:0] _zz_76_;
  wire [31:0] _zz_77_;
  wire [31:0] _zz_78_;
  wire [31:0] decode_INSTRUCTION;
  wire [31:0] _zz_79_;
  wire [31:0] writeBack_PC;
  wire [31:0] writeBack_INSTRUCTION;
  reg  decode_arbitration_haltItself;
  reg  decode_arbitration_haltByOther;
  reg  decode_arbitration_removeIt;
  wire  decode_arbitration_flushIt;
  reg  decode_arbitration_flushNext;
  reg  decode_arbitration_isValid;
  wire  decode_arbitration_isStuck;
  wire  decode_arbitration_isStuckByOthers;
  wire  decode_arbitration_isFlushed;
  wire  decode_arbitration_isMoving;
  wire  decode_arbitration_isFiring;
  reg  execute_arbitration_haltItself;
  reg  execute_arbitration_haltByOther;
  reg  execute_arbitration_removeIt;
  reg  execute_arbitration_flushIt;
  reg  execute_arbitration_flushNext;
  reg  execute_arbitration_isValid;
  wire  execute_arbitration_isStuck;
  wire  execute_arbitration_isStuckByOthers;
  wire  execute_arbitration_isFlushed;
  wire  execute_arbitration_isMoving;
  wire  execute_arbitration_isFiring;
  reg  memory_arbitration_haltItself;
  wire  memory_arbitration_haltByOther;
  reg  memory_arbitration_removeIt;
  wire  memory_arbitration_flushIt;
  reg  memory_arbitration_flushNext;
  reg  memory_arbitration_isValid;
  wire  memory_arbitration_isStuck;
  wire  memory_arbitration_isStuckByOthers;
  wire  memory_arbitration_isFlushed;
  wire  memory_arbitration_isMoving;
  wire  memory_arbitration_isFiring;
  wire  writeBack_arbitration_haltItself;
  wire  writeBack_arbitration_haltByOther;
  reg  writeBack_arbitration_removeIt;
  wire  writeBack_arbitration_flushIt;
  reg  writeBack_arbitration_flushNext;
  reg  writeBack_arbitration_isValid;
  wire  writeBack_arbitration_isStuck;
  wire  writeBack_arbitration_isStuckByOthers;
  wire  writeBack_arbitration_isFlushed;
  wire  writeBack_arbitration_isMoving;
  wire  writeBack_arbitration_isFiring;
  wire [31:0] lastStageInstruction /* verilator public */ ;
  wire [31:0] lastStagePc /* verilator public */ ;
  wire  lastStageIsValid /* verilator public */ ;
  wire  lastStageIsFiring /* verilator public */ ;
  reg  IBusCachedPlugin_fetcherHalt;
  reg  IBusCachedPlugin_fetcherflushIt;
  reg  IBusCachedPlugin_incomingInstruction;
  wire  IBusCachedPlugin_pcValids_0;
  wire  IBusCachedPlugin_pcValids_1;
  wire  IBusCachedPlugin_pcValids_2;
  wire  IBusCachedPlugin_pcValids_3;
  wire  IBusCachedPlugin_redoBranch_valid;
  wire [31:0] IBusCachedPlugin_redoBranch_payload;
  wire  BranchPlugin_jumpInterface_valid;
  wire [31:0] BranchPlugin_jumpInterface_payload;
  reg  CsrPlugin_jumpInterface_valid;
  reg [31:0] CsrPlugin_jumpInterface_payload;
  wire  CsrPlugin_exceptionPendings_0;
  wire  CsrPlugin_exceptionPendings_1;
  wire  CsrPlugin_exceptionPendings_2;
  wire  CsrPlugin_exceptionPendings_3;
  wire  contextSwitching;
  reg [1:0] CsrPlugin_privilege;
  reg  CsrPlugin_forceMachineWire;
  reg  CsrPlugin_selfException_valid;
  reg [3:0] CsrPlugin_selfException_payload_code;
  wire [31:0] CsrPlugin_selfException_payload_badAddr;
  reg  CsrPlugin_allowInterrupts;
  reg  CsrPlugin_allowException;
  reg  IBusCachedPlugin_injectionPort_valid;
  reg  IBusCachedPlugin_injectionPort_ready;
  wire [31:0] IBusCachedPlugin_injectionPort_payload;
  wire  IBusCachedPlugin_jump_pcLoad_valid;
  wire [31:0] IBusCachedPlugin_jump_pcLoad_payload;
  wire [2:0] _zz_80_;
  wire [2:0] _zz_81_;
  wire  _zz_82_;
  wire  _zz_83_;
  wire  IBusCachedPlugin_fetchPc_output_valid;
  wire  IBusCachedPlugin_fetchPc_output_ready;
  wire [31:0] IBusCachedPlugin_fetchPc_output_payload;
  reg [31:0] IBusCachedPlugin_fetchPc_pcReg /* verilator public */ ;
  reg  IBusCachedPlugin_fetchPc_corrected;
  reg  IBusCachedPlugin_fetchPc_pcRegPropagate;
  reg  IBusCachedPlugin_fetchPc_booted;
  reg  IBusCachedPlugin_fetchPc_inc;
  reg [31:0] IBusCachedPlugin_fetchPc_pc;
  wire  IBusCachedPlugin_iBusRsp_stages_0_input_valid;
  wire  IBusCachedPlugin_iBusRsp_stages_0_input_ready;
  wire [31:0] IBusCachedPlugin_iBusRsp_stages_0_input_payload;
  wire  IBusCachedPlugin_iBusRsp_stages_0_output_valid;
  wire  IBusCachedPlugin_iBusRsp_stages_0_output_ready;
  wire [31:0] IBusCachedPlugin_iBusRsp_stages_0_output_payload;
  reg  IBusCachedPlugin_iBusRsp_stages_0_halt;
  wire  IBusCachedPlugin_iBusRsp_stages_0_inputSample;
  wire  IBusCachedPlugin_iBusRsp_stages_1_input_valid;
  wire  IBusCachedPlugin_iBusRsp_stages_1_input_ready;
  wire [31:0] IBusCachedPlugin_iBusRsp_stages_1_input_payload;
  wire  IBusCachedPlugin_iBusRsp_stages_1_output_valid;
  wire  IBusCachedPlugin_iBusRsp_stages_1_output_ready;
  wire [31:0] IBusCachedPlugin_iBusRsp_stages_1_output_payload;
  reg  IBusCachedPlugin_iBusRsp_stages_1_halt;
  wire  IBusCachedPlugin_iBusRsp_stages_1_inputSample;
  wire  _zz_84_;
  wire  _zz_85_;
  wire  _zz_86_;
  wire  _zz_87_;
  reg  _zz_88_;
  reg  IBusCachedPlugin_iBusRsp_readyForError;
  wire  IBusCachedPlugin_iBusRsp_output_valid;
  wire  IBusCachedPlugin_iBusRsp_output_ready;
  wire [31:0] IBusCachedPlugin_iBusRsp_output_payload_pc;
  wire  IBusCachedPlugin_iBusRsp_output_payload_rsp_error;
  wire [31:0] IBusCachedPlugin_iBusRsp_output_payload_rsp_inst;
  wire  IBusCachedPlugin_iBusRsp_output_payload_isRvc;
  wire  IBusCachedPlugin_injector_decodeInput_valid;
  wire  IBusCachedPlugin_injector_decodeInput_ready;
  wire [31:0] IBusCachedPlugin_injector_decodeInput_payload_pc;
  wire  IBusCachedPlugin_injector_decodeInput_payload_rsp_error;
  wire [31:0] IBusCachedPlugin_injector_decodeInput_payload_rsp_inst;
  wire  IBusCachedPlugin_injector_decodeInput_payload_isRvc;
  reg  _zz_89_;
  reg [31:0] _zz_90_;
  reg  _zz_91_;
  reg [31:0] _zz_92_;
  reg  _zz_93_;
  reg  IBusCachedPlugin_injector_nextPcCalc_valids_0;
  reg  IBusCachedPlugin_injector_nextPcCalc_valids_1;
  reg  IBusCachedPlugin_injector_nextPcCalc_valids_2;
  reg  IBusCachedPlugin_injector_nextPcCalc_valids_3;
  reg  IBusCachedPlugin_injector_nextPcCalc_valids_4;
  reg  IBusCachedPlugin_injector_decodeRemoved;
  reg [31:0] IBusCachedPlugin_injector_formal_rawInDecode;
  wire [31:0] _zz_94_;
  reg [31:0] IBusCachedPlugin_rspCounter;
  wire  IBusCachedPlugin_s0_tightlyCoupledHit;
  reg  IBusCachedPlugin_s1_tightlyCoupledHit;
  wire  IBusCachedPlugin_rsp_iBusRspOutputHalt;
  reg  IBusCachedPlugin_rsp_redoFetch;
  wire  _zz_95_;
  reg  execute_DBusSimplePlugin_skipCmd;
  reg [31:0] _zz_96_;
  reg [3:0] _zz_97_;
  wire [3:0] execute_DBusSimplePlugin_formalMask;
  reg [31:0] writeBack_DBusSimplePlugin_rspShifted;
  wire  _zz_98_;
  reg [31:0] _zz_99_;
  wire  _zz_100_;
  reg [31:0] _zz_101_;
  reg [31:0] writeBack_DBusSimplePlugin_rspFormated;
  wire [27:0] _zz_102_;
  wire  _zz_103_;
  wire  _zz_104_;
  wire  _zz_105_;
  wire  _zz_106_;
  wire  _zz_107_;
  wire `EnvCtrlEnum_defaultEncoding_type _zz_108_;
  wire `Src2CtrlEnum_defaultEncoding_type _zz_109_;
  wire `BranchCtrlEnum_defaultEncoding_type _zz_110_;
  wire `AluCtrlEnum_defaultEncoding_type _zz_111_;
  wire `Src1CtrlEnum_defaultEncoding_type _zz_112_;
  wire `AluBitwiseCtrlEnum_defaultEncoding_type _zz_113_;
  wire `ShiftCtrlEnum_defaultEncoding_type _zz_114_;
  wire [4:0] decode_RegFilePlugin_regFileReadAddress1;
  wire [4:0] decode_RegFilePlugin_regFileReadAddress2;
  wire [31:0] decode_RegFilePlugin_rs1Data;
  wire [31:0] decode_RegFilePlugin_rs2Data;
  wire  lastStageRegFileWrite_valid /* verilator public */ ;
  wire [4:0] lastStageRegFileWrite_payload_address /* verilator public */ ;
  wire [31:0] lastStageRegFileWrite_payload_data /* verilator public */ ;
  reg [31:0] execute_IntAluPlugin_bitwise;
  reg [31:0] _zz_115_;
  reg [31:0] _zz_116_;
  wire  _zz_117_;
  reg [19:0] _zz_118_;
  wire  _zz_119_;
  reg [19:0] _zz_120_;
  reg [31:0] _zz_121_;
  reg [31:0] execute_SrcPlugin_addSub;
  wire  execute_SrcPlugin_less;
  reg  execute_LightShifterPlugin_isActive;
  wire  execute_LightShifterPlugin_isShift;
  reg [4:0] execute_LightShifterPlugin_amplitudeReg;
  wire [4:0] execute_LightShifterPlugin_amplitude;
  wire [31:0] execute_LightShifterPlugin_shiftInput;
  wire  execute_LightShifterPlugin_done;
  reg [31:0] _zz_122_;
  wire  execute_BranchPlugin_eq;
  wire [2:0] _zz_123_;
  reg  _zz_124_;
  reg  _zz_125_;
  wire [31:0] execute_BranchPlugin_branch_src1;
  wire  _zz_126_;
  reg [10:0] _zz_127_;
  wire  _zz_128_;
  reg [19:0] _zz_129_;
  wire  _zz_130_;
  reg [18:0] _zz_131_;
  reg [31:0] _zz_132_;
  wire [31:0] execute_BranchPlugin_branch_src2;
  wire [31:0] execute_BranchPlugin_branchAdder;
  reg  _zz_133_;
  reg  _zz_134_;
  wire  _zz_135_;
  reg  _zz_136_;
  reg [4:0] _zz_137_;
  wire [1:0] CsrPlugin_misa_base;
  wire [25:0] CsrPlugin_misa_extensions;
  reg [1:0] CsrPlugin_mtvec_mode;
  reg [29:0] CsrPlugin_mtvec_base;
  reg [31:0] CsrPlugin_mepc;
  reg  CsrPlugin_mstatus_MIE;
  reg  CsrPlugin_mstatus_MPIE;
  reg [1:0] CsrPlugin_mstatus_MPP;
  reg  CsrPlugin_mip_MEIP;
  reg  CsrPlugin_mip_MTIP;
  reg  CsrPlugin_mip_MSIP;
  reg  CsrPlugin_mie_MEIE;
  reg  CsrPlugin_mie_MTIE;
  reg  CsrPlugin_mie_MSIE;
  reg  CsrPlugin_mcause_interrupt;
  reg [3:0] CsrPlugin_mcause_exceptionCode;
  reg [31:0] CsrPlugin_mtval;
  reg [63:0] CsrPlugin_mcycle = 64'b0000000000000000000000000000000000000000000000000000000000000000;
  reg [63:0] CsrPlugin_minstret = 64'b0000000000000000000000000000000000000000000000000000000000000000;
  wire  _zz_138_;
  wire  _zz_139_;
  wire  _zz_140_;
  wire  CsrPlugin_exceptionPortCtrl_exceptionValids_decode;
  reg  CsrPlugin_exceptionPortCtrl_exceptionValids_execute;
  reg  CsrPlugin_exceptionPortCtrl_exceptionValids_memory;
  reg  CsrPlugin_exceptionPortCtrl_exceptionValids_writeBack;
  wire  CsrPlugin_exceptionPortCtrl_exceptionValidsRegs_decode;
  reg  CsrPlugin_exceptionPortCtrl_exceptionValidsRegs_execute;
  reg  CsrPlugin_exceptionPortCtrl_exceptionValidsRegs_memory;
  reg  CsrPlugin_exceptionPortCtrl_exceptionValidsRegs_writeBack;
  reg [3:0] CsrPlugin_exceptionPortCtrl_exceptionContext_code;
  reg [31:0] CsrPlugin_exceptionPortCtrl_exceptionContext_badAddr;
  wire [1:0] CsrPlugin_exceptionPortCtrl_exceptionTargetPrivilegeUncapped;
  wire [1:0] CsrPlugin_exceptionPortCtrl_exceptionTargetPrivilege;
  reg  CsrPlugin_interrupt_valid;
  reg [3:0] CsrPlugin_interrupt_code /* verilator public */ ;
  reg [1:0] CsrPlugin_interrupt_targetPrivilege;
  wire  CsrPlugin_exception;
  wire  CsrPlugin_lastStageWasWfi;
  reg  CsrPlugin_pipelineLiberator_done;
  wire  CsrPlugin_interruptJump /* verilator public */ ;
  reg  CsrPlugin_hadException;
  reg [1:0] CsrPlugin_targetPrivilege;
  reg [3:0] CsrPlugin_trapCause;
  reg [1:0] CsrPlugin_xtvec_mode;
  reg [29:0] CsrPlugin_xtvec_base;
  wire  execute_CsrPlugin_inWfi /* verilator public */ ;
  reg  execute_CsrPlugin_wfiWake;
  wire  execute_CsrPlugin_blockedBySideEffects;
  reg  execute_CsrPlugin_illegalAccess;
  reg  execute_CsrPlugin_illegalInstruction;
  reg [31:0] execute_CsrPlugin_readData;
  wire  execute_CsrPlugin_writeInstruction;
  wire  execute_CsrPlugin_readInstruction;
  wire  execute_CsrPlugin_writeEnable;
  wire  execute_CsrPlugin_readEnable;
  wire [31:0] execute_CsrPlugin_readToWriteData;
  reg [31:0] execute_CsrPlugin_writeData;
  wire [11:0] execute_CsrPlugin_csrAddress;
  reg  DebugPlugin_firstCycle;
  reg  DebugPlugin_secondCycle;
  reg  DebugPlugin_resetIt;
  reg  DebugPlugin_haltIt;
  reg  DebugPlugin_stepIt;
  reg  DebugPlugin_isPipBusy;
  reg  DebugPlugin_godmode;
  reg  DebugPlugin_haltedByBreak;
  reg  DebugPlugin_hardwareBreakpoints_0_valid;
  reg [30:0] DebugPlugin_hardwareBreakpoints_0_pc;
  reg  DebugPlugin_hardwareBreakpoints_1_valid;
  reg [30:0] DebugPlugin_hardwareBreakpoints_1_pc;
  reg  DebugPlugin_hardwareBreakpoints_2_valid;
  reg [30:0] DebugPlugin_hardwareBreakpoints_2_pc;
  reg  DebugPlugin_hardwareBreakpoints_3_valid;
  reg [30:0] DebugPlugin_hardwareBreakpoints_3_pc;
  reg [31:0] DebugPlugin_busReadDataReg;
  reg  _zz_141_;
  reg  DebugPlugin_resetIt_regNext;
  reg [31:0] execute_to_memory_BRANCH_CALC;
  reg  decode_to_execute_MEMORY_ENABLE;
  reg  execute_to_memory_MEMORY_ENABLE;
  reg  memory_to_writeBack_MEMORY_ENABLE;
  reg  decode_to_execute_CSR_READ_OPCODE;
  reg  decode_to_execute_SRC_LESS_UNSIGNED;
  reg  decode_to_execute_SRC_USE_SUB_LESS;
  reg  decode_to_execute_SRC2_FORCE_ZERO;
  reg  decode_to_execute_IS_CSR;
  reg [31:0] decode_to_execute_SRC1;
  reg  execute_to_memory_BRANCH_DO;
  reg [31:0] decode_to_execute_RS2;
  reg  decode_to_execute_DO_EBREAK;
  reg [31:0] decode_to_execute_RS1;
  reg [31:0] memory_to_writeBack_MEMORY_READ_DATA;
  reg [31:0] execute_to_memory_REGFILE_WRITE_DATA;
  reg [31:0] memory_to_writeBack_REGFILE_WRITE_DATA;
  reg  decode_to_execute_REGFILE_WRITE_VALID;
  reg  execute_to_memory_REGFILE_WRITE_VALID;
  reg  memory_to_writeBack_REGFILE_WRITE_VALID;
  reg [31:0] decode_to_execute_FORMAL_PC_NEXT;
  reg [31:0] execute_to_memory_FORMAL_PC_NEXT;
  reg [31:0] memory_to_writeBack_FORMAL_PC_NEXT;
  reg `BranchCtrlEnum_defaultEncoding_type decode_to_execute_BRANCH_CTRL;
  reg [31:0] decode_to_execute_INSTRUCTION;
  reg [31:0] execute_to_memory_INSTRUCTION;
  reg [31:0] memory_to_writeBack_INSTRUCTION;
  reg `AluCtrlEnum_defaultEncoding_type decode_to_execute_ALU_CTRL;
  reg `ShiftCtrlEnum_defaultEncoding_type decode_to_execute_SHIFT_CTRL;
  reg `EnvCtrlEnum_defaultEncoding_type decode_to_execute_ENV_CTRL;
  reg `EnvCtrlEnum_defaultEncoding_type execute_to_memory_ENV_CTRL;
  reg `EnvCtrlEnum_defaultEncoding_type memory_to_writeBack_ENV_CTRL;
  reg  decode_to_execute_CSR_WRITE_OPCODE;
  reg [31:0] decode_to_execute_PC;
  reg [31:0] execute_to_memory_PC;
  reg [31:0] memory_to_writeBack_PC;
  reg  decode_to_execute_BYPASSABLE_MEMORY_STAGE;
  reg  execute_to_memory_BYPASSABLE_MEMORY_STAGE;
  reg [31:0] decode_to_execute_SRC2;
  reg  decode_to_execute_BYPASSABLE_EXECUTE_STAGE;
  reg  decode_to_execute_MEMORY_STORE;
  reg  execute_to_memory_MEMORY_STORE;
  reg  memory_to_writeBack_MEMORY_STORE;
  reg [1:0] execute_to_memory_MEMORY_ADDRESS_LOW;
  reg [1:0] memory_to_writeBack_MEMORY_ADDRESS_LOW;
  reg `AluBitwiseCtrlEnum_defaultEncoding_type decode_to_execute_ALU_BITWISE_CTRL;
  reg [2:0] _zz_142_;
  `ifndef SYNTHESIS
  reg [39:0] decode_ALU_BITWISE_CTRL_string;
  reg [39:0] _zz_1__string;
  reg [39:0] _zz_2__string;
  reg [39:0] _zz_3__string;
  reg [39:0] _zz_4__string;
  reg [39:0] _zz_5__string;
  reg [39:0] _zz_6__string;
  reg [39:0] _zz_7__string;
  reg [39:0] decode_ENV_CTRL_string;
  reg [39:0] _zz_8__string;
  reg [39:0] _zz_9__string;
  reg [39:0] _zz_10__string;
  reg [71:0] decode_SHIFT_CTRL_string;
  reg [71:0] _zz_11__string;
  reg [71:0] _zz_12__string;
  reg [71:0] _zz_13__string;
  reg [63:0] decode_ALU_CTRL_string;
  reg [63:0] _zz_14__string;
  reg [63:0] _zz_15__string;
  reg [63:0] _zz_16__string;
  reg [31:0] decode_BRANCH_CTRL_string;
  reg [31:0] _zz_17__string;
  reg [31:0] _zz_18__string;
  reg [31:0] _zz_19__string;
  reg [39:0] memory_ENV_CTRL_string;
  reg [39:0] _zz_21__string;
  reg [39:0] execute_ENV_CTRL_string;
  reg [39:0] _zz_22__string;
  reg [39:0] writeBack_ENV_CTRL_string;
  reg [39:0] _zz_25__string;
  reg [31:0] execute_BRANCH_CTRL_string;
  reg [31:0] _zz_27__string;
  reg [71:0] execute_SHIFT_CTRL_string;
  reg [71:0] _zz_30__string;
  reg [23:0] decode_SRC2_CTRL_string;
  reg [23:0] _zz_36__string;
  reg [95:0] decode_SRC1_CTRL_string;
  reg [95:0] _zz_39__string;
  reg [63:0] execute_ALU_CTRL_string;
  reg [63:0] _zz_42__string;
  reg [39:0] execute_ALU_BITWISE_CTRL_string;
  reg [39:0] _zz_44__string;
  reg [71:0] _zz_52__string;
  reg [39:0] _zz_57__string;
  reg [95:0] _zz_58__string;
  reg [63:0] _zz_61__string;
  reg [31:0] _zz_63__string;
  reg [23:0] _zz_65__string;
  reg [39:0] _zz_66__string;
  reg [39:0] _zz_108__string;
  reg [23:0] _zz_109__string;
  reg [31:0] _zz_110__string;
  reg [63:0] _zz_111__string;
  reg [95:0] _zz_112__string;
  reg [39:0] _zz_113__string;
  reg [71:0] _zz_114__string;
  reg [31:0] decode_to_execute_BRANCH_CTRL_string;
  reg [63:0] decode_to_execute_ALU_CTRL_string;
  reg [71:0] decode_to_execute_SHIFT_CTRL_string;
  reg [39:0] decode_to_execute_ENV_CTRL_string;
  reg [39:0] execute_to_memory_ENV_CTRL_string;
  reg [39:0] memory_to_writeBack_ENV_CTRL_string;
  reg [39:0] decode_to_execute_ALU_BITWISE_CTRL_string;
  `endif

  reg [31:0] RegFilePlugin_regFile [0:31] /* verilator public */ ;
  assign _zz_163_ = ((execute_arbitration_isValid && execute_LightShifterPlugin_isShift) && (execute_SRC2[4 : 0] != (5'b00000)));
  assign _zz_164_ = (execute_arbitration_isValid && execute_IS_CSR);
  assign _zz_165_ = ((_zz_145_ && IBusCachedPlugin_cache_io_cpu_fetch_cacheMiss) && (! _zz_73_));
  assign _zz_166_ = ((_zz_145_ && IBusCachedPlugin_cache_io_cpu_fetch_mmuRefilling) && (! 1'b0));
  assign _zz_167_ = (! execute_arbitration_isStuckByOthers);
  assign _zz_168_ = (execute_arbitration_isValid && execute_DO_EBREAK);
  assign _zz_169_ = (({writeBack_arbitration_isValid,memory_arbitration_isValid} != (2'b00)) == 1'b0);
  assign _zz_170_ = (CsrPlugin_hadException || CsrPlugin_interruptJump);
  assign _zz_171_ = (writeBack_arbitration_isValid && (writeBack_ENV_CTRL == `EnvCtrlEnum_defaultEncoding_XRET));
  assign _zz_172_ = (DebugPlugin_stepIt && IBusCachedPlugin_incomingInstruction);
  assign _zz_173_ = writeBack_INSTRUCTION[29 : 28];
  assign _zz_174_ = (! IBusCachedPlugin_iBusRsp_readyForError);
  assign _zz_175_ = (writeBack_arbitration_isValid && writeBack_REGFILE_WRITE_VALID);
  assign _zz_176_ = (1'b1 || (! 1'b1));
  assign _zz_177_ = (memory_arbitration_isValid && memory_REGFILE_WRITE_VALID);
  assign _zz_178_ = (1'b1 || (! memory_BYPASSABLE_MEMORY_STAGE));
  assign _zz_179_ = (execute_arbitration_isValid && execute_REGFILE_WRITE_VALID);
  assign _zz_180_ = (1'b1 || (! execute_BYPASSABLE_EXECUTE_STAGE));
  assign _zz_181_ = (execute_arbitration_isValid && (execute_ENV_CTRL == `EnvCtrlEnum_defaultEncoding_ECALL));
  assign _zz_182_ = debug_bus_cmd_payload_address[7 : 2];
  assign _zz_183_ = (CsrPlugin_mstatus_MIE || (CsrPlugin_privilege < (2'b11)));
  assign _zz_184_ = ((_zz_138_ && 1'b1) && (! 1'b0));
  assign _zz_185_ = ((_zz_139_ && 1'b1) && (! 1'b0));
  assign _zz_186_ = ((_zz_140_ && 1'b1) && (! 1'b0));
  assign _zz_187_ = writeBack_INSTRUCTION[13 : 12];
  assign _zz_188_ = execute_INSTRUCTION[13];
  assign _zz_189_ = (_zz_80_ - (3'b001));
  assign _zz_190_ = {IBusCachedPlugin_fetchPc_inc,(2'b00)};
  assign _zz_191_ = {29'd0, _zz_190_};
  assign _zz_192_ = _zz_102_[0 : 0];
  assign _zz_193_ = _zz_102_[1 : 1];
  assign _zz_194_ = _zz_102_[2 : 2];
  assign _zz_195_ = _zz_102_[7 : 7];
  assign _zz_196_ = _zz_102_[11 : 11];
  assign _zz_197_ = _zz_102_[14 : 14];
  assign _zz_198_ = _zz_102_[15 : 15];
  assign _zz_199_ = _zz_102_[20 : 20];
  assign _zz_200_ = _zz_102_[21 : 21];
  assign _zz_201_ = _zz_102_[22 : 22];
  assign _zz_202_ = _zz_102_[23 : 23];
  assign _zz_203_ = _zz_102_[26 : 26];
  assign _zz_204_ = _zz_102_[27 : 27];
  assign _zz_205_ = execute_SRC_LESS;
  assign _zz_206_ = (3'b100);
  assign _zz_207_ = decode_INSTRUCTION[19 : 15];
  assign _zz_208_ = decode_INSTRUCTION[31 : 20];
  assign _zz_209_ = {decode_INSTRUCTION[31 : 25],decode_INSTRUCTION[11 : 7]};
  assign _zz_210_ = ($signed(_zz_211_) + $signed(_zz_214_));
  assign _zz_211_ = ($signed(_zz_212_) + $signed(_zz_213_));
  assign _zz_212_ = execute_SRC1;
  assign _zz_213_ = (execute_SRC_USE_SUB_LESS ? (~ execute_SRC2) : execute_SRC2);
  assign _zz_214_ = (execute_SRC_USE_SUB_LESS ? _zz_215_ : _zz_216_);
  assign _zz_215_ = (32'b00000000000000000000000000000001);
  assign _zz_216_ = (32'b00000000000000000000000000000000);
  assign _zz_217_ = (_zz_218_ >>> 1);
  assign _zz_218_ = {((execute_SHIFT_CTRL == `ShiftCtrlEnum_defaultEncoding_SRA_1) && execute_LightShifterPlugin_shiftInput[31]),execute_LightShifterPlugin_shiftInput};
  assign _zz_219_ = {{{execute_INSTRUCTION[31],execute_INSTRUCTION[19 : 12]},execute_INSTRUCTION[20]},execute_INSTRUCTION[30 : 21]};
  assign _zz_220_ = execute_INSTRUCTION[31 : 20];
  assign _zz_221_ = {{{execute_INSTRUCTION[31],execute_INSTRUCTION[7]},execute_INSTRUCTION[30 : 25]},execute_INSTRUCTION[11 : 8]};
  assign _zz_222_ = debug_bus_cmd_payload_data[0 : 0];
  assign _zz_223_ = debug_bus_cmd_payload_data[0 : 0];
  assign _zz_224_ = debug_bus_cmd_payload_data[0 : 0];
  assign _zz_225_ = debug_bus_cmd_payload_data[0 : 0];
  assign _zz_226_ = (decode_PC >>> 1);
  assign _zz_227_ = (decode_PC >>> 1);
  assign _zz_228_ = (decode_PC >>> 1);
  assign _zz_229_ = (decode_PC >>> 1);
  assign _zz_230_ = execute_CsrPlugin_writeData[7 : 7];
  assign _zz_231_ = execute_CsrPlugin_writeData[3 : 3];
  assign _zz_232_ = execute_CsrPlugin_writeData[3 : 3];
  assign _zz_233_ = execute_CsrPlugin_writeData[11 : 11];
  assign _zz_234_ = execute_CsrPlugin_writeData[7 : 7];
  assign _zz_235_ = execute_CsrPlugin_writeData[3 : 3];
  assign _zz_236_ = 1'b1;
  assign _zz_237_ = 1'b1;
  assign _zz_238_ = {_zz_83_,_zz_82_};
  assign _zz_239_ = (decode_INSTRUCTION & (32'b00000000000000000000000000110100));
  assign _zz_240_ = (32'b00000000000000000000000000100000);
  assign _zz_241_ = (decode_INSTRUCTION & (32'b00000000000000000000000001100100));
  assign _zz_242_ = (32'b00000000000000000000000000100000);
  assign _zz_243_ = ((decode_INSTRUCTION & (32'b00000000000000000000000001100100)) == (32'b00000000000000000000000000100100));
  assign _zz_244_ = ((decode_INSTRUCTION & (32'b00000000000000000011000001010100)) == (32'b00000000000000000001000000010000));
  assign _zz_245_ = ((decode_INSTRUCTION & (32'b00000000000000000111000001010100)) == (32'b00000000000000000101000000010000));
  assign _zz_246_ = {(_zz_251_ == _zz_252_),(_zz_253_ == _zz_254_)};
  assign _zz_247_ = (2'b00);
  assign _zz_248_ = ((_zz_255_ == _zz_256_) != (1'b0));
  assign _zz_249_ = (_zz_257_ != (1'b0));
  assign _zz_250_ = {(_zz_258_ != _zz_259_),{_zz_260_,{_zz_261_,_zz_262_}}};
  assign _zz_251_ = (decode_INSTRUCTION & (32'b01000000000000000011000001010100));
  assign _zz_252_ = (32'b01000000000000000001000000010000);
  assign _zz_253_ = (decode_INSTRUCTION & (32'b00000000000000000111000001010100));
  assign _zz_254_ = (32'b00000000000000000001000000010000);
  assign _zz_255_ = (decode_INSTRUCTION & (32'b00010000000100000011000001010000));
  assign _zz_256_ = (32'b00000000000100000000000001010000);
  assign _zz_257_ = ((decode_INSTRUCTION & (32'b00000000000000000000000000010000)) == (32'b00000000000000000000000000010000));
  assign _zz_258_ = {(_zz_263_ == _zz_264_),(_zz_265_ == _zz_266_)};
  assign _zz_259_ = (2'b00);
  assign _zz_260_ = ((_zz_267_ == _zz_268_) != (1'b0));
  assign _zz_261_ = (_zz_269_ != (1'b0));
  assign _zz_262_ = {(_zz_270_ != _zz_271_),{_zz_272_,{_zz_273_,_zz_274_}}};
  assign _zz_263_ = (decode_INSTRUCTION & (32'b00000000000000000010000000010000));
  assign _zz_264_ = (32'b00000000000000000010000000000000);
  assign _zz_265_ = (decode_INSTRUCTION & (32'b00000000000000000101000000000000));
  assign _zz_266_ = (32'b00000000000000000001000000000000);
  assign _zz_267_ = (decode_INSTRUCTION & (32'b00000000000000000000000000100000));
  assign _zz_268_ = (32'b00000000000000000000000000100000);
  assign _zz_269_ = ((decode_INSTRUCTION & (32'b00000000000000000001000000000000)) == (32'b00000000000000000001000000000000));
  assign _zz_270_ = ((decode_INSTRUCTION & (32'b00000000000000000011000000000000)) == (32'b00000000000000000010000000000000));
  assign _zz_271_ = (1'b0);
  assign _zz_272_ = ({(_zz_275_ == _zz_276_),_zz_107_} != (2'b00));
  assign _zz_273_ = ({_zz_277_,_zz_107_} != (2'b00));
  assign _zz_274_ = {(_zz_106_ != (1'b0)),{(_zz_278_ != _zz_279_),{_zz_280_,{_zz_281_,_zz_282_}}}};
  assign _zz_275_ = (decode_INSTRUCTION & (32'b00000000000000000000000000010100));
  assign _zz_276_ = (32'b00000000000000000000000000000100);
  assign _zz_277_ = ((decode_INSTRUCTION & (32'b00000000000000000000000001000100)) == (32'b00000000000000000000000000000100));
  assign _zz_278_ = {_zz_105_,{_zz_283_,{_zz_284_,_zz_285_}}};
  assign _zz_279_ = (6'b000000);
  assign _zz_280_ = ({_zz_286_,_zz_287_} != (2'b00));
  assign _zz_281_ = (_zz_103_ != (1'b0));
  assign _zz_282_ = {(_zz_288_ != _zz_289_),{_zz_290_,{_zz_291_,_zz_292_}}};
  assign _zz_283_ = ((decode_INSTRUCTION & _zz_293_) == (32'b00000000000000000001000000010000));
  assign _zz_284_ = (_zz_294_ == _zz_295_);
  assign _zz_285_ = {_zz_106_,{_zz_296_,_zz_297_}};
  assign _zz_286_ = ((decode_INSTRUCTION & _zz_298_) == (32'b00000000000000000110000000000000));
  assign _zz_287_ = ((decode_INSTRUCTION & _zz_299_) == (32'b00000000000000000100000000000000));
  assign _zz_288_ = {_zz_300_,{_zz_301_,_zz_302_}};
  assign _zz_289_ = (3'b000);
  assign _zz_290_ = ({_zz_303_,_zz_304_} != (2'b00));
  assign _zz_291_ = (_zz_305_ != _zz_306_);
  assign _zz_292_ = {_zz_307_,{_zz_308_,_zz_309_}};
  assign _zz_293_ = (32'b00000000000000000001000000010000);
  assign _zz_294_ = (decode_INSTRUCTION & (32'b00000000000000000010000000010000));
  assign _zz_295_ = (32'b00000000000000000010000000010000);
  assign _zz_296_ = (_zz_310_ == _zz_311_);
  assign _zz_297_ = (_zz_312_ == _zz_313_);
  assign _zz_298_ = (32'b00000000000000000110000000000100);
  assign _zz_299_ = (32'b00000000000000000101000000000100);
  assign _zz_300_ = ((decode_INSTRUCTION & _zz_314_) == (32'b00000000000000000000000001000000));
  assign _zz_301_ = (_zz_315_ == _zz_316_);
  assign _zz_302_ = (_zz_317_ == _zz_318_);
  assign _zz_303_ = _zz_105_;
  assign _zz_304_ = (_zz_319_ == _zz_320_);
  assign _zz_305_ = (_zz_321_ == _zz_322_);
  assign _zz_306_ = (1'b0);
  assign _zz_307_ = ({_zz_323_,_zz_324_} != (2'b00));
  assign _zz_308_ = (_zz_325_ != _zz_326_);
  assign _zz_309_ = {_zz_327_,{_zz_328_,_zz_329_}};
  assign _zz_310_ = (decode_INSTRUCTION & (32'b00000000000000000000000000001100));
  assign _zz_311_ = (32'b00000000000000000000000000000100);
  assign _zz_312_ = (decode_INSTRUCTION & (32'b00000000000000000000000000101000));
  assign _zz_313_ = (32'b00000000000000000000000000000000);
  assign _zz_314_ = (32'b00000000000000000000000001000100);
  assign _zz_315_ = (decode_INSTRUCTION & (32'b00000000000000000010000000010100));
  assign _zz_316_ = (32'b00000000000000000010000000010000);
  assign _zz_317_ = (decode_INSTRUCTION & (32'b01000000000000000100000000110100));
  assign _zz_318_ = (32'b01000000000000000000000000110000);
  assign _zz_319_ = (decode_INSTRUCTION & (32'b00000000000000000000000000011100));
  assign _zz_320_ = (32'b00000000000000000000000000000100);
  assign _zz_321_ = (decode_INSTRUCTION & (32'b00000000000000000000000001011000));
  assign _zz_322_ = (32'b00000000000000000000000001000000);
  assign _zz_323_ = ((decode_INSTRUCTION & _zz_330_) == (32'b00000000000000000000000001000000));
  assign _zz_324_ = ((decode_INSTRUCTION & _zz_331_) == (32'b00000000000000000000000001000000));
  assign _zz_325_ = ((decode_INSTRUCTION & _zz_332_) == (32'b00000000000000000000000000000000));
  assign _zz_326_ = (1'b0);
  assign _zz_327_ = ({_zz_104_,_zz_333_} != (2'b00));
  assign _zz_328_ = ({_zz_334_,_zz_335_} != (2'b00));
  assign _zz_329_ = {(_zz_336_ != _zz_337_),{_zz_338_,{_zz_339_,_zz_340_}}};
  assign _zz_330_ = (32'b00000000000000000000000001010000);
  assign _zz_331_ = (32'b00000000000100000011000001000000);
  assign _zz_332_ = (32'b00000000000000000000000001011000);
  assign _zz_333_ = ((decode_INSTRUCTION & (32'b00000000000000000000000001110000)) == (32'b00000000000000000000000000100000));
  assign _zz_334_ = _zz_104_;
  assign _zz_335_ = ((decode_INSTRUCTION & (32'b00000000000000000000000000100000)) == (32'b00000000000000000000000000000000));
  assign _zz_336_ = ((decode_INSTRUCTION & (32'b00010000000100000011000001010000)) == (32'b00000000000000000000000001010000));
  assign _zz_337_ = (1'b0);
  assign _zz_338_ = (((decode_INSTRUCTION & _zz_341_) == (32'b00010000000000000000000001010000)) != (1'b0));
  assign _zz_339_ = ((_zz_342_ == _zz_343_) != (1'b0));
  assign _zz_340_ = {({_zz_344_,_zz_345_} != (2'b00)),({_zz_346_,_zz_347_} != (4'b0000))};
  assign _zz_341_ = (32'b00010000010000000011000001010000);
  assign _zz_342_ = (decode_INSTRUCTION & (32'b00000000000000000001000001001000));
  assign _zz_343_ = (32'b00000000000000000001000000001000);
  assign _zz_344_ = ((decode_INSTRUCTION & (32'b00000000000000000001000001010000)) == (32'b00000000000000000001000001010000));
  assign _zz_345_ = ((decode_INSTRUCTION & (32'b00000000000000000010000001010000)) == (32'b00000000000000000010000001010000));
  assign _zz_346_ = ((decode_INSTRUCTION & (32'b00000000000000000000000001000100)) == (32'b00000000000000000000000000000000));
  assign _zz_347_ = {((decode_INSTRUCTION & (32'b00000000000000000000000000011000)) == (32'b00000000000000000000000000000000)),{_zz_103_,((decode_INSTRUCTION & (32'b00000000000000000101000000000100)) == (32'b00000000000000000001000000000000))}};
  initial begin
    $readmemb("Ice40up5kbevn.v_toplevel_system_cpu_cpu_RegFilePlugin_regFile.bin",RegFilePlugin_regFile);
  end
  always @ (posedge clk_12M) begin
    if(_zz_47_) begin
      RegFilePlugin_regFile[lastStageRegFileWrite_payload_address] <= lastStageRegFileWrite_payload_data;
    end
  end

  always @ (posedge clk_12M) begin
    if(_zz_236_) begin
      _zz_160_ <= RegFilePlugin_regFile[decode_RegFilePlugin_regFileReadAddress1];
    end
  end

  always @ (posedge clk_12M) begin
    if(_zz_237_) begin
      _zz_161_ <= RegFilePlugin_regFile[decode_RegFilePlugin_regFileReadAddress2];
    end
  end

  InstructionCache IBusCachedPlugin_cache ( 
    .io_flush(_zz_143_),
    .io_cpu_prefetch_isValid(_zz_144_),
    .io_cpu_prefetch_haltIt(IBusCachedPlugin_cache_io_cpu_prefetch_haltIt),
    .io_cpu_prefetch_pc(IBusCachedPlugin_iBusRsp_stages_0_input_payload),
    .io_cpu_fetch_isValid(_zz_145_),
    .io_cpu_fetch_isStuck(_zz_146_),
    .io_cpu_fetch_isRemoved(IBusCachedPlugin_fetcherflushIt),
    .io_cpu_fetch_pc(IBusCachedPlugin_iBusRsp_stages_1_input_payload),
    .io_cpu_fetch_data(IBusCachedPlugin_cache_io_cpu_fetch_data),
    .io_cpu_fetch_dataBypassValid(IBusCachedPlugin_s1_tightlyCoupledHit),
    .io_cpu_fetch_dataBypass(_zz_147_),
    .io_cpu_fetch_mmuBus_cmd_isValid(IBusCachedPlugin_cache_io_cpu_fetch_mmuBus_cmd_isValid),
    .io_cpu_fetch_mmuBus_cmd_virtualAddress(IBusCachedPlugin_cache_io_cpu_fetch_mmuBus_cmd_virtualAddress),
    .io_cpu_fetch_mmuBus_cmd_bypassTranslation(IBusCachedPlugin_cache_io_cpu_fetch_mmuBus_cmd_bypassTranslation),
    .io_cpu_fetch_mmuBus_rsp_physicalAddress(IBusCachedPlugin_cache_io_cpu_fetch_mmuBus_cmd_virtualAddress),
    .io_cpu_fetch_mmuBus_rsp_isIoAccess(_zz_148_),
    .io_cpu_fetch_mmuBus_rsp_allowRead(_zz_149_),
    .io_cpu_fetch_mmuBus_rsp_allowWrite(_zz_150_),
    .io_cpu_fetch_mmuBus_rsp_allowExecute(_zz_151_),
    .io_cpu_fetch_mmuBus_rsp_exception(_zz_152_),
    .io_cpu_fetch_mmuBus_rsp_refilling(_zz_153_),
    .io_cpu_fetch_mmuBus_end(IBusCachedPlugin_cache_io_cpu_fetch_mmuBus_end),
    .io_cpu_fetch_mmuBus_busy(_zz_154_),
    .io_cpu_fetch_physicalAddress(IBusCachedPlugin_cache_io_cpu_fetch_physicalAddress),
    .io_cpu_fetch_cacheMiss(IBusCachedPlugin_cache_io_cpu_fetch_cacheMiss),
    .io_cpu_fetch_error(IBusCachedPlugin_cache_io_cpu_fetch_error),
    .io_cpu_fetch_mmuRefilling(IBusCachedPlugin_cache_io_cpu_fetch_mmuRefilling),
    .io_cpu_fetch_mmuException(IBusCachedPlugin_cache_io_cpu_fetch_mmuException),
    .io_cpu_fetch_isUser(_zz_155_),
    .io_cpu_fetch_haltIt(IBusCachedPlugin_cache_io_cpu_fetch_haltIt),
    .io_cpu_decode_isValid(_zz_156_),
    .io_cpu_decode_isStuck(_zz_157_),
    .io_cpu_decode_pc(_zz_158_),
    .io_cpu_decode_physicalAddress(IBusCachedPlugin_cache_io_cpu_decode_physicalAddress),
    .io_cpu_decode_data(IBusCachedPlugin_cache_io_cpu_decode_data),
    .io_cpu_fill_valid(_zz_159_),
    .io_cpu_fill_payload(IBusCachedPlugin_cache_io_cpu_fetch_physicalAddress),
    .io_mem_cmd_valid(IBusCachedPlugin_cache_io_mem_cmd_valid),
    .io_mem_cmd_ready(iBus_cmd_ready),
    .io_mem_cmd_payload_address(IBusCachedPlugin_cache_io_mem_cmd_payload_address),
    .io_mem_cmd_payload_size(IBusCachedPlugin_cache_io_mem_cmd_payload_size),
    .io_mem_rsp_valid(iBus_rsp_valid),
    .io_mem_rsp_payload_data(iBus_rsp_payload_data),
    .io_mem_rsp_payload_error(iBus_rsp_payload_error),
    .clk_12M(clk_12M),
    .clockCtrl_systemReset(clockCtrl_systemReset) 
  );
  always @(*) begin
    case(_zz_238_)
      2'b00 : begin
        _zz_162_ = CsrPlugin_jumpInterface_payload;
      end
      2'b01 : begin
        _zz_162_ = BranchPlugin_jumpInterface_payload;
      end
      default : begin
        _zz_162_ = IBusCachedPlugin_redoBranch_payload;
      end
    endcase
  end

  `ifndef SYNTHESIS
  always @(*) begin
    case(decode_ALU_BITWISE_CTRL)
      `AluBitwiseCtrlEnum_defaultEncoding_XOR_1 : decode_ALU_BITWISE_CTRL_string = "XOR_1";
      `AluBitwiseCtrlEnum_defaultEncoding_OR_1 : decode_ALU_BITWISE_CTRL_string = "OR_1 ";
      `AluBitwiseCtrlEnum_defaultEncoding_AND_1 : decode_ALU_BITWISE_CTRL_string = "AND_1";
      default : decode_ALU_BITWISE_CTRL_string = "?????";
    endcase
  end
  always @(*) begin
    case(_zz_1_)
      `AluBitwiseCtrlEnum_defaultEncoding_XOR_1 : _zz_1__string = "XOR_1";
      `AluBitwiseCtrlEnum_defaultEncoding_OR_1 : _zz_1__string = "OR_1 ";
      `AluBitwiseCtrlEnum_defaultEncoding_AND_1 : _zz_1__string = "AND_1";
      default : _zz_1__string = "?????";
    endcase
  end
  always @(*) begin
    case(_zz_2_)
      `AluBitwiseCtrlEnum_defaultEncoding_XOR_1 : _zz_2__string = "XOR_1";
      `AluBitwiseCtrlEnum_defaultEncoding_OR_1 : _zz_2__string = "OR_1 ";
      `AluBitwiseCtrlEnum_defaultEncoding_AND_1 : _zz_2__string = "AND_1";
      default : _zz_2__string = "?????";
    endcase
  end
  always @(*) begin
    case(_zz_3_)
      `AluBitwiseCtrlEnum_defaultEncoding_XOR_1 : _zz_3__string = "XOR_1";
      `AluBitwiseCtrlEnum_defaultEncoding_OR_1 : _zz_3__string = "OR_1 ";
      `AluBitwiseCtrlEnum_defaultEncoding_AND_1 : _zz_3__string = "AND_1";
      default : _zz_3__string = "?????";
    endcase
  end
  always @(*) begin
    case(_zz_4_)
      `EnvCtrlEnum_defaultEncoding_NONE : _zz_4__string = "NONE ";
      `EnvCtrlEnum_defaultEncoding_XRET : _zz_4__string = "XRET ";
      `EnvCtrlEnum_defaultEncoding_ECALL : _zz_4__string = "ECALL";
      default : _zz_4__string = "?????";
    endcase
  end
  always @(*) begin
    case(_zz_5_)
      `EnvCtrlEnum_defaultEncoding_NONE : _zz_5__string = "NONE ";
      `EnvCtrlEnum_defaultEncoding_XRET : _zz_5__string = "XRET ";
      `EnvCtrlEnum_defaultEncoding_ECALL : _zz_5__string = "ECALL";
      default : _zz_5__string = "?????";
    endcase
  end
  always @(*) begin
    case(_zz_6_)
      `EnvCtrlEnum_defaultEncoding_NONE : _zz_6__string = "NONE ";
      `EnvCtrlEnum_defaultEncoding_XRET : _zz_6__string = "XRET ";
      `EnvCtrlEnum_defaultEncoding_ECALL : _zz_6__string = "ECALL";
      default : _zz_6__string = "?????";
    endcase
  end
  always @(*) begin
    case(_zz_7_)
      `EnvCtrlEnum_defaultEncoding_NONE : _zz_7__string = "NONE ";
      `EnvCtrlEnum_defaultEncoding_XRET : _zz_7__string = "XRET ";
      `EnvCtrlEnum_defaultEncoding_ECALL : _zz_7__string = "ECALL";
      default : _zz_7__string = "?????";
    endcase
  end
  always @(*) begin
    case(decode_ENV_CTRL)
      `EnvCtrlEnum_defaultEncoding_NONE : decode_ENV_CTRL_string = "NONE ";
      `EnvCtrlEnum_defaultEncoding_XRET : decode_ENV_CTRL_string = "XRET ";
      `EnvCtrlEnum_defaultEncoding_ECALL : decode_ENV_CTRL_string = "ECALL";
      default : decode_ENV_CTRL_string = "?????";
    endcase
  end
  always @(*) begin
    case(_zz_8_)
      `EnvCtrlEnum_defaultEncoding_NONE : _zz_8__string = "NONE ";
      `EnvCtrlEnum_defaultEncoding_XRET : _zz_8__string = "XRET ";
      `EnvCtrlEnum_defaultEncoding_ECALL : _zz_8__string = "ECALL";
      default : _zz_8__string = "?????";
    endcase
  end
  always @(*) begin
    case(_zz_9_)
      `EnvCtrlEnum_defaultEncoding_NONE : _zz_9__string = "NONE ";
      `EnvCtrlEnum_defaultEncoding_XRET : _zz_9__string = "XRET ";
      `EnvCtrlEnum_defaultEncoding_ECALL : _zz_9__string = "ECALL";
      default : _zz_9__string = "?????";
    endcase
  end
  always @(*) begin
    case(_zz_10_)
      `EnvCtrlEnum_defaultEncoding_NONE : _zz_10__string = "NONE ";
      `EnvCtrlEnum_defaultEncoding_XRET : _zz_10__string = "XRET ";
      `EnvCtrlEnum_defaultEncoding_ECALL : _zz_10__string = "ECALL";
      default : _zz_10__string = "?????";
    endcase
  end
  always @(*) begin
    case(decode_SHIFT_CTRL)
      `ShiftCtrlEnum_defaultEncoding_DISABLE_1 : decode_SHIFT_CTRL_string = "DISABLE_1";
      `ShiftCtrlEnum_defaultEncoding_SLL_1 : decode_SHIFT_CTRL_string = "SLL_1    ";
      `ShiftCtrlEnum_defaultEncoding_SRL_1 : decode_SHIFT_CTRL_string = "SRL_1    ";
      `ShiftCtrlEnum_defaultEncoding_SRA_1 : decode_SHIFT_CTRL_string = "SRA_1    ";
      default : decode_SHIFT_CTRL_string = "?????????";
    endcase
  end
  always @(*) begin
    case(_zz_11_)
      `ShiftCtrlEnum_defaultEncoding_DISABLE_1 : _zz_11__string = "DISABLE_1";
      `ShiftCtrlEnum_defaultEncoding_SLL_1 : _zz_11__string = "SLL_1    ";
      `ShiftCtrlEnum_defaultEncoding_SRL_1 : _zz_11__string = "SRL_1    ";
      `ShiftCtrlEnum_defaultEncoding_SRA_1 : _zz_11__string = "SRA_1    ";
      default : _zz_11__string = "?????????";
    endcase
  end
  always @(*) begin
    case(_zz_12_)
      `ShiftCtrlEnum_defaultEncoding_DISABLE_1 : _zz_12__string = "DISABLE_1";
      `ShiftCtrlEnum_defaultEncoding_SLL_1 : _zz_12__string = "SLL_1    ";
      `ShiftCtrlEnum_defaultEncoding_SRL_1 : _zz_12__string = "SRL_1    ";
      `ShiftCtrlEnum_defaultEncoding_SRA_1 : _zz_12__string = "SRA_1    ";
      default : _zz_12__string = "?????????";
    endcase
  end
  always @(*) begin
    case(_zz_13_)
      `ShiftCtrlEnum_defaultEncoding_DISABLE_1 : _zz_13__string = "DISABLE_1";
      `ShiftCtrlEnum_defaultEncoding_SLL_1 : _zz_13__string = "SLL_1    ";
      `ShiftCtrlEnum_defaultEncoding_SRL_1 : _zz_13__string = "SRL_1    ";
      `ShiftCtrlEnum_defaultEncoding_SRA_1 : _zz_13__string = "SRA_1    ";
      default : _zz_13__string = "?????????";
    endcase
  end
  always @(*) begin
    case(decode_ALU_CTRL)
      `AluCtrlEnum_defaultEncoding_ADD_SUB : decode_ALU_CTRL_string = "ADD_SUB ";
      `AluCtrlEnum_defaultEncoding_SLT_SLTU : decode_ALU_CTRL_string = "SLT_SLTU";
      `AluCtrlEnum_defaultEncoding_BITWISE : decode_ALU_CTRL_string = "BITWISE ";
      default : decode_ALU_CTRL_string = "????????";
    endcase
  end
  always @(*) begin
    case(_zz_14_)
      `AluCtrlEnum_defaultEncoding_ADD_SUB : _zz_14__string = "ADD_SUB ";
      `AluCtrlEnum_defaultEncoding_SLT_SLTU : _zz_14__string = "SLT_SLTU";
      `AluCtrlEnum_defaultEncoding_BITWISE : _zz_14__string = "BITWISE ";
      default : _zz_14__string = "????????";
    endcase
  end
  always @(*) begin
    case(_zz_15_)
      `AluCtrlEnum_defaultEncoding_ADD_SUB : _zz_15__string = "ADD_SUB ";
      `AluCtrlEnum_defaultEncoding_SLT_SLTU : _zz_15__string = "SLT_SLTU";
      `AluCtrlEnum_defaultEncoding_BITWISE : _zz_15__string = "BITWISE ";
      default : _zz_15__string = "????????";
    endcase
  end
  always @(*) begin
    case(_zz_16_)
      `AluCtrlEnum_defaultEncoding_ADD_SUB : _zz_16__string = "ADD_SUB ";
      `AluCtrlEnum_defaultEncoding_SLT_SLTU : _zz_16__string = "SLT_SLTU";
      `AluCtrlEnum_defaultEncoding_BITWISE : _zz_16__string = "BITWISE ";
      default : _zz_16__string = "????????";
    endcase
  end
  always @(*) begin
    case(decode_BRANCH_CTRL)
      `BranchCtrlEnum_defaultEncoding_INC : decode_BRANCH_CTRL_string = "INC ";
      `BranchCtrlEnum_defaultEncoding_B : decode_BRANCH_CTRL_string = "B   ";
      `BranchCtrlEnum_defaultEncoding_JAL : decode_BRANCH_CTRL_string = "JAL ";
      `BranchCtrlEnum_defaultEncoding_JALR : decode_BRANCH_CTRL_string = "JALR";
      default : decode_BRANCH_CTRL_string = "????";
    endcase
  end
  always @(*) begin
    case(_zz_17_)
      `BranchCtrlEnum_defaultEncoding_INC : _zz_17__string = "INC ";
      `BranchCtrlEnum_defaultEncoding_B : _zz_17__string = "B   ";
      `BranchCtrlEnum_defaultEncoding_JAL : _zz_17__string = "JAL ";
      `BranchCtrlEnum_defaultEncoding_JALR : _zz_17__string = "JALR";
      default : _zz_17__string = "????";
    endcase
  end
  always @(*) begin
    case(_zz_18_)
      `BranchCtrlEnum_defaultEncoding_INC : _zz_18__string = "INC ";
      `BranchCtrlEnum_defaultEncoding_B : _zz_18__string = "B   ";
      `BranchCtrlEnum_defaultEncoding_JAL : _zz_18__string = "JAL ";
      `BranchCtrlEnum_defaultEncoding_JALR : _zz_18__string = "JALR";
      default : _zz_18__string = "????";
    endcase
  end
  always @(*) begin
    case(_zz_19_)
      `BranchCtrlEnum_defaultEncoding_INC : _zz_19__string = "INC ";
      `BranchCtrlEnum_defaultEncoding_B : _zz_19__string = "B   ";
      `BranchCtrlEnum_defaultEncoding_JAL : _zz_19__string = "JAL ";
      `BranchCtrlEnum_defaultEncoding_JALR : _zz_19__string = "JALR";
      default : _zz_19__string = "????";
    endcase
  end
  always @(*) begin
    case(memory_ENV_CTRL)
      `EnvCtrlEnum_defaultEncoding_NONE : memory_ENV_CTRL_string = "NONE ";
      `EnvCtrlEnum_defaultEncoding_XRET : memory_ENV_CTRL_string = "XRET ";
      `EnvCtrlEnum_defaultEncoding_ECALL : memory_ENV_CTRL_string = "ECALL";
      default : memory_ENV_CTRL_string = "?????";
    endcase
  end
  always @(*) begin
    case(_zz_21_)
      `EnvCtrlEnum_defaultEncoding_NONE : _zz_21__string = "NONE ";
      `EnvCtrlEnum_defaultEncoding_XRET : _zz_21__string = "XRET ";
      `EnvCtrlEnum_defaultEncoding_ECALL : _zz_21__string = "ECALL";
      default : _zz_21__string = "?????";
    endcase
  end
  always @(*) begin
    case(execute_ENV_CTRL)
      `EnvCtrlEnum_defaultEncoding_NONE : execute_ENV_CTRL_string = "NONE ";
      `EnvCtrlEnum_defaultEncoding_XRET : execute_ENV_CTRL_string = "XRET ";
      `EnvCtrlEnum_defaultEncoding_ECALL : execute_ENV_CTRL_string = "ECALL";
      default : execute_ENV_CTRL_string = "?????";
    endcase
  end
  always @(*) begin
    case(_zz_22_)
      `EnvCtrlEnum_defaultEncoding_NONE : _zz_22__string = "NONE ";
      `EnvCtrlEnum_defaultEncoding_XRET : _zz_22__string = "XRET ";
      `EnvCtrlEnum_defaultEncoding_ECALL : _zz_22__string = "ECALL";
      default : _zz_22__string = "?????";
    endcase
  end
  always @(*) begin
    case(writeBack_ENV_CTRL)
      `EnvCtrlEnum_defaultEncoding_NONE : writeBack_ENV_CTRL_string = "NONE ";
      `EnvCtrlEnum_defaultEncoding_XRET : writeBack_ENV_CTRL_string = "XRET ";
      `EnvCtrlEnum_defaultEncoding_ECALL : writeBack_ENV_CTRL_string = "ECALL";
      default : writeBack_ENV_CTRL_string = "?????";
    endcase
  end
  always @(*) begin
    case(_zz_25_)
      `EnvCtrlEnum_defaultEncoding_NONE : _zz_25__string = "NONE ";
      `EnvCtrlEnum_defaultEncoding_XRET : _zz_25__string = "XRET ";
      `EnvCtrlEnum_defaultEncoding_ECALL : _zz_25__string = "ECALL";
      default : _zz_25__string = "?????";
    endcase
  end
  always @(*) begin
    case(execute_BRANCH_CTRL)
      `BranchCtrlEnum_defaultEncoding_INC : execute_BRANCH_CTRL_string = "INC ";
      `BranchCtrlEnum_defaultEncoding_B : execute_BRANCH_CTRL_string = "B   ";
      `BranchCtrlEnum_defaultEncoding_JAL : execute_BRANCH_CTRL_string = "JAL ";
      `BranchCtrlEnum_defaultEncoding_JALR : execute_BRANCH_CTRL_string = "JALR";
      default : execute_BRANCH_CTRL_string = "????";
    endcase
  end
  always @(*) begin
    case(_zz_27_)
      `BranchCtrlEnum_defaultEncoding_INC : _zz_27__string = "INC ";
      `BranchCtrlEnum_defaultEncoding_B : _zz_27__string = "B   ";
      `BranchCtrlEnum_defaultEncoding_JAL : _zz_27__string = "JAL ";
      `BranchCtrlEnum_defaultEncoding_JALR : _zz_27__string = "JALR";
      default : _zz_27__string = "????";
    endcase
  end
  always @(*) begin
    case(execute_SHIFT_CTRL)
      `ShiftCtrlEnum_defaultEncoding_DISABLE_1 : execute_SHIFT_CTRL_string = "DISABLE_1";
      `ShiftCtrlEnum_defaultEncoding_SLL_1 : execute_SHIFT_CTRL_string = "SLL_1    ";
      `ShiftCtrlEnum_defaultEncoding_SRL_1 : execute_SHIFT_CTRL_string = "SRL_1    ";
      `ShiftCtrlEnum_defaultEncoding_SRA_1 : execute_SHIFT_CTRL_string = "SRA_1    ";
      default : execute_SHIFT_CTRL_string = "?????????";
    endcase
  end
  always @(*) begin
    case(_zz_30_)
      `ShiftCtrlEnum_defaultEncoding_DISABLE_1 : _zz_30__string = "DISABLE_1";
      `ShiftCtrlEnum_defaultEncoding_SLL_1 : _zz_30__string = "SLL_1    ";
      `ShiftCtrlEnum_defaultEncoding_SRL_1 : _zz_30__string = "SRL_1    ";
      `ShiftCtrlEnum_defaultEncoding_SRA_1 : _zz_30__string = "SRA_1    ";
      default : _zz_30__string = "?????????";
    endcase
  end
  always @(*) begin
    case(decode_SRC2_CTRL)
      `Src2CtrlEnum_defaultEncoding_RS : decode_SRC2_CTRL_string = "RS ";
      `Src2CtrlEnum_defaultEncoding_IMI : decode_SRC2_CTRL_string = "IMI";
      `Src2CtrlEnum_defaultEncoding_IMS : decode_SRC2_CTRL_string = "IMS";
      `Src2CtrlEnum_defaultEncoding_PC : decode_SRC2_CTRL_string = "PC ";
      default : decode_SRC2_CTRL_string = "???";
    endcase
  end
  always @(*) begin
    case(_zz_36_)
      `Src2CtrlEnum_defaultEncoding_RS : _zz_36__string = "RS ";
      `Src2CtrlEnum_defaultEncoding_IMI : _zz_36__string = "IMI";
      `Src2CtrlEnum_defaultEncoding_IMS : _zz_36__string = "IMS";
      `Src2CtrlEnum_defaultEncoding_PC : _zz_36__string = "PC ";
      default : _zz_36__string = "???";
    endcase
  end
  always @(*) begin
    case(decode_SRC1_CTRL)
      `Src1CtrlEnum_defaultEncoding_RS : decode_SRC1_CTRL_string = "RS          ";
      `Src1CtrlEnum_defaultEncoding_IMU : decode_SRC1_CTRL_string = "IMU         ";
      `Src1CtrlEnum_defaultEncoding_PC_INCREMENT : decode_SRC1_CTRL_string = "PC_INCREMENT";
      `Src1CtrlEnum_defaultEncoding_URS1 : decode_SRC1_CTRL_string = "URS1        ";
      default : decode_SRC1_CTRL_string = "????????????";
    endcase
  end
  always @(*) begin
    case(_zz_39_)
      `Src1CtrlEnum_defaultEncoding_RS : _zz_39__string = "RS          ";
      `Src1CtrlEnum_defaultEncoding_IMU : _zz_39__string = "IMU         ";
      `Src1CtrlEnum_defaultEncoding_PC_INCREMENT : _zz_39__string = "PC_INCREMENT";
      `Src1CtrlEnum_defaultEncoding_URS1 : _zz_39__string = "URS1        ";
      default : _zz_39__string = "????????????";
    endcase
  end
  always @(*) begin
    case(execute_ALU_CTRL)
      `AluCtrlEnum_defaultEncoding_ADD_SUB : execute_ALU_CTRL_string = "ADD_SUB ";
      `AluCtrlEnum_defaultEncoding_SLT_SLTU : execute_ALU_CTRL_string = "SLT_SLTU";
      `AluCtrlEnum_defaultEncoding_BITWISE : execute_ALU_CTRL_string = "BITWISE ";
      default : execute_ALU_CTRL_string = "????????";
    endcase
  end
  always @(*) begin
    case(_zz_42_)
      `AluCtrlEnum_defaultEncoding_ADD_SUB : _zz_42__string = "ADD_SUB ";
      `AluCtrlEnum_defaultEncoding_SLT_SLTU : _zz_42__string = "SLT_SLTU";
      `AluCtrlEnum_defaultEncoding_BITWISE : _zz_42__string = "BITWISE ";
      default : _zz_42__string = "????????";
    endcase
  end
  always @(*) begin
    case(execute_ALU_BITWISE_CTRL)
      `AluBitwiseCtrlEnum_defaultEncoding_XOR_1 : execute_ALU_BITWISE_CTRL_string = "XOR_1";
      `AluBitwiseCtrlEnum_defaultEncoding_OR_1 : execute_ALU_BITWISE_CTRL_string = "OR_1 ";
      `AluBitwiseCtrlEnum_defaultEncoding_AND_1 : execute_ALU_BITWISE_CTRL_string = "AND_1";
      default : execute_ALU_BITWISE_CTRL_string = "?????";
    endcase
  end
  always @(*) begin
    case(_zz_44_)
      `AluBitwiseCtrlEnum_defaultEncoding_XOR_1 : _zz_44__string = "XOR_1";
      `AluBitwiseCtrlEnum_defaultEncoding_OR_1 : _zz_44__string = "OR_1 ";
      `AluBitwiseCtrlEnum_defaultEncoding_AND_1 : _zz_44__string = "AND_1";
      default : _zz_44__string = "?????";
    endcase
  end
  always @(*) begin
    case(_zz_52_)
      `ShiftCtrlEnum_defaultEncoding_DISABLE_1 : _zz_52__string = "DISABLE_1";
      `ShiftCtrlEnum_defaultEncoding_SLL_1 : _zz_52__string = "SLL_1    ";
      `ShiftCtrlEnum_defaultEncoding_SRL_1 : _zz_52__string = "SRL_1    ";
      `ShiftCtrlEnum_defaultEncoding_SRA_1 : _zz_52__string = "SRA_1    ";
      default : _zz_52__string = "?????????";
    endcase
  end
  always @(*) begin
    case(_zz_57_)
      `AluBitwiseCtrlEnum_defaultEncoding_XOR_1 : _zz_57__string = "XOR_1";
      `AluBitwiseCtrlEnum_defaultEncoding_OR_1 : _zz_57__string = "OR_1 ";
      `AluBitwiseCtrlEnum_defaultEncoding_AND_1 : _zz_57__string = "AND_1";
      default : _zz_57__string = "?????";
    endcase
  end
  always @(*) begin
    case(_zz_58_)
      `Src1CtrlEnum_defaultEncoding_RS : _zz_58__string = "RS          ";
      `Src1CtrlEnum_defaultEncoding_IMU : _zz_58__string = "IMU         ";
      `Src1CtrlEnum_defaultEncoding_PC_INCREMENT : _zz_58__string = "PC_INCREMENT";
      `Src1CtrlEnum_defaultEncoding_URS1 : _zz_58__string = "URS1        ";
      default : _zz_58__string = "????????????";
    endcase
  end
  always @(*) begin
    case(_zz_61_)
      `AluCtrlEnum_defaultEncoding_ADD_SUB : _zz_61__string = "ADD_SUB ";
      `AluCtrlEnum_defaultEncoding_SLT_SLTU : _zz_61__string = "SLT_SLTU";
      `AluCtrlEnum_defaultEncoding_BITWISE : _zz_61__string = "BITWISE ";
      default : _zz_61__string = "????????";
    endcase
  end
  always @(*) begin
    case(_zz_63_)
      `BranchCtrlEnum_defaultEncoding_INC : _zz_63__string = "INC ";
      `BranchCtrlEnum_defaultEncoding_B : _zz_63__string = "B   ";
      `BranchCtrlEnum_defaultEncoding_JAL : _zz_63__string = "JAL ";
      `BranchCtrlEnum_defaultEncoding_JALR : _zz_63__string = "JALR";
      default : _zz_63__string = "????";
    endcase
  end
  always @(*) begin
    case(_zz_65_)
      `Src2CtrlEnum_defaultEncoding_RS : _zz_65__string = "RS ";
      `Src2CtrlEnum_defaultEncoding_IMI : _zz_65__string = "IMI";
      `Src2CtrlEnum_defaultEncoding_IMS : _zz_65__string = "IMS";
      `Src2CtrlEnum_defaultEncoding_PC : _zz_65__string = "PC ";
      default : _zz_65__string = "???";
    endcase
  end
  always @(*) begin
    case(_zz_66_)
      `EnvCtrlEnum_defaultEncoding_NONE : _zz_66__string = "NONE ";
      `EnvCtrlEnum_defaultEncoding_XRET : _zz_66__string = "XRET ";
      `EnvCtrlEnum_defaultEncoding_ECALL : _zz_66__string = "ECALL";
      default : _zz_66__string = "?????";
    endcase
  end
  always @(*) begin
    case(_zz_108_)
      `EnvCtrlEnum_defaultEncoding_NONE : _zz_108__string = "NONE ";
      `EnvCtrlEnum_defaultEncoding_XRET : _zz_108__string = "XRET ";
      `EnvCtrlEnum_defaultEncoding_ECALL : _zz_108__string = "ECALL";
      default : _zz_108__string = "?????";
    endcase
  end
  always @(*) begin
    case(_zz_109_)
      `Src2CtrlEnum_defaultEncoding_RS : _zz_109__string = "RS ";
      `Src2CtrlEnum_defaultEncoding_IMI : _zz_109__string = "IMI";
      `Src2CtrlEnum_defaultEncoding_IMS : _zz_109__string = "IMS";
      `Src2CtrlEnum_defaultEncoding_PC : _zz_109__string = "PC ";
      default : _zz_109__string = "???";
    endcase
  end
  always @(*) begin
    case(_zz_110_)
      `BranchCtrlEnum_defaultEncoding_INC : _zz_110__string = "INC ";
      `BranchCtrlEnum_defaultEncoding_B : _zz_110__string = "B   ";
      `BranchCtrlEnum_defaultEncoding_JAL : _zz_110__string = "JAL ";
      `BranchCtrlEnum_defaultEncoding_JALR : _zz_110__string = "JALR";
      default : _zz_110__string = "????";
    endcase
  end
  always @(*) begin
    case(_zz_111_)
      `AluCtrlEnum_defaultEncoding_ADD_SUB : _zz_111__string = "ADD_SUB ";
      `AluCtrlEnum_defaultEncoding_SLT_SLTU : _zz_111__string = "SLT_SLTU";
      `AluCtrlEnum_defaultEncoding_BITWISE : _zz_111__string = "BITWISE ";
      default : _zz_111__string = "????????";
    endcase
  end
  always @(*) begin
    case(_zz_112_)
      `Src1CtrlEnum_defaultEncoding_RS : _zz_112__string = "RS          ";
      `Src1CtrlEnum_defaultEncoding_IMU : _zz_112__string = "IMU         ";
      `Src1CtrlEnum_defaultEncoding_PC_INCREMENT : _zz_112__string = "PC_INCREMENT";
      `Src1CtrlEnum_defaultEncoding_URS1 : _zz_112__string = "URS1        ";
      default : _zz_112__string = "????????????";
    endcase
  end
  always @(*) begin
    case(_zz_113_)
      `AluBitwiseCtrlEnum_defaultEncoding_XOR_1 : _zz_113__string = "XOR_1";
      `AluBitwiseCtrlEnum_defaultEncoding_OR_1 : _zz_113__string = "OR_1 ";
      `AluBitwiseCtrlEnum_defaultEncoding_AND_1 : _zz_113__string = "AND_1";
      default : _zz_113__string = "?????";
    endcase
  end
  always @(*) begin
    case(_zz_114_)
      `ShiftCtrlEnum_defaultEncoding_DISABLE_1 : _zz_114__string = "DISABLE_1";
      `ShiftCtrlEnum_defaultEncoding_SLL_1 : _zz_114__string = "SLL_1    ";
      `ShiftCtrlEnum_defaultEncoding_SRL_1 : _zz_114__string = "SRL_1    ";
      `ShiftCtrlEnum_defaultEncoding_SRA_1 : _zz_114__string = "SRA_1    ";
      default : _zz_114__string = "?????????";
    endcase
  end
  always @(*) begin
    case(decode_to_execute_BRANCH_CTRL)
      `BranchCtrlEnum_defaultEncoding_INC : decode_to_execute_BRANCH_CTRL_string = "INC ";
      `BranchCtrlEnum_defaultEncoding_B : decode_to_execute_BRANCH_CTRL_string = "B   ";
      `BranchCtrlEnum_defaultEncoding_JAL : decode_to_execute_BRANCH_CTRL_string = "JAL ";
      `BranchCtrlEnum_defaultEncoding_JALR : decode_to_execute_BRANCH_CTRL_string = "JALR";
      default : decode_to_execute_BRANCH_CTRL_string = "????";
    endcase
  end
  always @(*) begin
    case(decode_to_execute_ALU_CTRL)
      `AluCtrlEnum_defaultEncoding_ADD_SUB : decode_to_execute_ALU_CTRL_string = "ADD_SUB ";
      `AluCtrlEnum_defaultEncoding_SLT_SLTU : decode_to_execute_ALU_CTRL_string = "SLT_SLTU";
      `AluCtrlEnum_defaultEncoding_BITWISE : decode_to_execute_ALU_CTRL_string = "BITWISE ";
      default : decode_to_execute_ALU_CTRL_string = "????????";
    endcase
  end
  always @(*) begin
    case(decode_to_execute_SHIFT_CTRL)
      `ShiftCtrlEnum_defaultEncoding_DISABLE_1 : decode_to_execute_SHIFT_CTRL_string = "DISABLE_1";
      `ShiftCtrlEnum_defaultEncoding_SLL_1 : decode_to_execute_SHIFT_CTRL_string = "SLL_1    ";
      `ShiftCtrlEnum_defaultEncoding_SRL_1 : decode_to_execute_SHIFT_CTRL_string = "SRL_1    ";
      `ShiftCtrlEnum_defaultEncoding_SRA_1 : decode_to_execute_SHIFT_CTRL_string = "SRA_1    ";
      default : decode_to_execute_SHIFT_CTRL_string = "?????????";
    endcase
  end
  always @(*) begin
    case(decode_to_execute_ENV_CTRL)
      `EnvCtrlEnum_defaultEncoding_NONE : decode_to_execute_ENV_CTRL_string = "NONE ";
      `EnvCtrlEnum_defaultEncoding_XRET : decode_to_execute_ENV_CTRL_string = "XRET ";
      `EnvCtrlEnum_defaultEncoding_ECALL : decode_to_execute_ENV_CTRL_string = "ECALL";
      default : decode_to_execute_ENV_CTRL_string = "?????";
    endcase
  end
  always @(*) begin
    case(execute_to_memory_ENV_CTRL)
      `EnvCtrlEnum_defaultEncoding_NONE : execute_to_memory_ENV_CTRL_string = "NONE ";
      `EnvCtrlEnum_defaultEncoding_XRET : execute_to_memory_ENV_CTRL_string = "XRET ";
      `EnvCtrlEnum_defaultEncoding_ECALL : execute_to_memory_ENV_CTRL_string = "ECALL";
      default : execute_to_memory_ENV_CTRL_string = "?????";
    endcase
  end
  always @(*) begin
    case(memory_to_writeBack_ENV_CTRL)
      `EnvCtrlEnum_defaultEncoding_NONE : memory_to_writeBack_ENV_CTRL_string = "NONE ";
      `EnvCtrlEnum_defaultEncoding_XRET : memory_to_writeBack_ENV_CTRL_string = "XRET ";
      `EnvCtrlEnum_defaultEncoding_ECALL : memory_to_writeBack_ENV_CTRL_string = "ECALL";
      default : memory_to_writeBack_ENV_CTRL_string = "?????";
    endcase
  end
  always @(*) begin
    case(decode_to_execute_ALU_BITWISE_CTRL)
      `AluBitwiseCtrlEnum_defaultEncoding_XOR_1 : decode_to_execute_ALU_BITWISE_CTRL_string = "XOR_1";
      `AluBitwiseCtrlEnum_defaultEncoding_OR_1 : decode_to_execute_ALU_BITWISE_CTRL_string = "OR_1 ";
      `AluBitwiseCtrlEnum_defaultEncoding_AND_1 : decode_to_execute_ALU_BITWISE_CTRL_string = "AND_1";
      default : decode_to_execute_ALU_BITWISE_CTRL_string = "?????";
    endcase
  end
  `endif

  assign decode_ALU_BITWISE_CTRL = _zz_1_;
  assign _zz_2_ = _zz_3_;
  assign memory_MEMORY_ADDRESS_LOW = execute_to_memory_MEMORY_ADDRESS_LOW;
  assign execute_MEMORY_ADDRESS_LOW = _zz_72_;
  assign decode_MEMORY_STORE = _zz_56_;
  assign decode_BYPASSABLE_EXECUTE_STAGE = _zz_59_;
  assign decode_SRC2 = _zz_37_;
  assign execute_BYPASSABLE_MEMORY_STAGE = decode_to_execute_BYPASSABLE_MEMORY_STAGE;
  assign decode_BYPASSABLE_MEMORY_STAGE = _zz_54_;
  assign memory_PC = execute_to_memory_PC;
  assign decode_CSR_WRITE_OPCODE = _zz_24_;
  assign _zz_4_ = _zz_5_;
  assign _zz_6_ = _zz_7_;
  assign decode_ENV_CTRL = _zz_8_;
  assign _zz_9_ = _zz_10_;
  assign decode_SHIFT_CTRL = _zz_11_;
  assign _zz_12_ = _zz_13_;
  assign decode_ALU_CTRL = _zz_14_;
  assign _zz_15_ = _zz_16_;
  assign decode_BRANCH_CTRL = _zz_17_;
  assign _zz_18_ = _zz_19_;
  assign writeBack_FORMAL_PC_NEXT = memory_to_writeBack_FORMAL_PC_NEXT;
  assign memory_FORMAL_PC_NEXT = execute_to_memory_FORMAL_PC_NEXT;
  assign execute_FORMAL_PC_NEXT = decode_to_execute_FORMAL_PC_NEXT;
  assign decode_FORMAL_PC_NEXT = _zz_76_;
  assign writeBack_REGFILE_WRITE_DATA = memory_to_writeBack_REGFILE_WRITE_DATA;
  assign execute_REGFILE_WRITE_DATA = _zz_43_;
  assign memory_MEMORY_READ_DATA = _zz_71_;
  assign decode_RS1 = _zz_49_;
  assign decode_DO_EBREAK = _zz_20_;
  assign decode_RS2 = _zz_48_;
  assign execute_BRANCH_DO = _zz_28_;
  assign decode_SRC1 = _zz_40_;
  assign decode_IS_CSR = _zz_68_;
  assign decode_SRC2_FORCE_ZERO = _zz_41_;
  assign decode_SRC_LESS_UNSIGNED = _zz_55_;
  assign decode_CSR_READ_OPCODE = _zz_23_;
  assign decode_MEMORY_ENABLE = _zz_64_;
  assign execute_BRANCH_CALC = _zz_26_;
  assign execute_DO_EBREAK = decode_to_execute_DO_EBREAK;
  assign decode_IS_EBREAK = _zz_53_;
  assign execute_CSR_READ_OPCODE = decode_to_execute_CSR_READ_OPCODE;
  assign execute_CSR_WRITE_OPCODE = decode_to_execute_CSR_WRITE_OPCODE;
  assign execute_IS_CSR = decode_to_execute_IS_CSR;
  assign memory_ENV_CTRL = _zz_21_;
  assign execute_ENV_CTRL = _zz_22_;
  assign writeBack_ENV_CTRL = _zz_25_;
  assign decode_RS2_USE = _zz_50_;
  assign decode_RS1_USE = _zz_69_;
  assign execute_REGFILE_WRITE_VALID = decode_to_execute_REGFILE_WRITE_VALID;
  assign execute_BYPASSABLE_EXECUTE_STAGE = decode_to_execute_BYPASSABLE_EXECUTE_STAGE;
  assign memory_REGFILE_WRITE_VALID = execute_to_memory_REGFILE_WRITE_VALID;
  assign memory_INSTRUCTION = execute_to_memory_INSTRUCTION;
  assign memory_BYPASSABLE_MEMORY_STAGE = execute_to_memory_BYPASSABLE_MEMORY_STAGE;
  assign writeBack_REGFILE_WRITE_VALID = memory_to_writeBack_REGFILE_WRITE_VALID;
  assign memory_BRANCH_CALC = execute_to_memory_BRANCH_CALC;
  assign memory_BRANCH_DO = execute_to_memory_BRANCH_DO;
  assign execute_PC = decode_to_execute_PC;
  assign execute_RS1 = decode_to_execute_RS1;
  assign execute_BRANCH_CTRL = _zz_27_;
  always @ (*) begin
    _zz_29_ = execute_REGFILE_WRITE_DATA;
    if(_zz_163_)begin
      _zz_29_ = _zz_122_;
    end
    if(_zz_164_)begin
      _zz_29_ = execute_CsrPlugin_readData;
    end
  end

  assign memory_REGFILE_WRITE_DATA = execute_to_memory_REGFILE_WRITE_DATA;
  assign execute_SHIFT_CTRL = _zz_30_;
  assign execute_SRC_LESS_UNSIGNED = decode_to_execute_SRC_LESS_UNSIGNED;
  assign execute_SRC2_FORCE_ZERO = decode_to_execute_SRC2_FORCE_ZERO;
  assign execute_SRC_USE_SUB_LESS = decode_to_execute_SRC_USE_SUB_LESS;
  assign _zz_34_ = decode_PC;
  assign _zz_35_ = decode_RS2;
  assign decode_SRC2_CTRL = _zz_36_;
  assign _zz_38_ = decode_RS1;
  assign decode_SRC1_CTRL = _zz_39_;
  assign decode_SRC_USE_SUB_LESS = _zz_62_;
  assign decode_SRC_ADD_ZERO = _zz_51_;
  assign execute_SRC_ADD_SUB = _zz_33_;
  assign execute_SRC_LESS = _zz_31_;
  assign execute_ALU_CTRL = _zz_42_;
  assign execute_SRC2 = decode_to_execute_SRC2;
  assign execute_SRC1 = decode_to_execute_SRC1;
  assign execute_ALU_BITWISE_CTRL = _zz_44_;
  assign _zz_45_ = writeBack_INSTRUCTION;
  assign _zz_46_ = writeBack_REGFILE_WRITE_VALID;
  always @ (*) begin
    _zz_47_ = 1'b0;
    if(lastStageRegFileWrite_valid)begin
      _zz_47_ = 1'b1;
    end
  end

  assign decode_INSTRUCTION_ANTICIPATED = _zz_79_;
  always @ (*) begin
    decode_REGFILE_WRITE_VALID = _zz_60_;
    if((decode_INSTRUCTION[11 : 7] == (5'b00000)))begin
      decode_REGFILE_WRITE_VALID = 1'b0;
    end
  end

  assign writeBack_MEMORY_STORE = memory_to_writeBack_MEMORY_STORE;
  always @ (*) begin
    _zz_70_ = writeBack_REGFILE_WRITE_DATA;
    if((writeBack_arbitration_isValid && writeBack_MEMORY_ENABLE))begin
      _zz_70_ = writeBack_DBusSimplePlugin_rspFormated;
    end
  end

  assign writeBack_MEMORY_ENABLE = memory_to_writeBack_MEMORY_ENABLE;
  assign writeBack_MEMORY_ADDRESS_LOW = memory_to_writeBack_MEMORY_ADDRESS_LOW;
  assign writeBack_MEMORY_READ_DATA = memory_to_writeBack_MEMORY_READ_DATA;
  assign memory_MEMORY_STORE = execute_to_memory_MEMORY_STORE;
  assign memory_MEMORY_ENABLE = execute_to_memory_MEMORY_ENABLE;
  assign execute_SRC_ADD = _zz_32_;
  assign execute_RS2 = decode_to_execute_RS2;
  assign execute_INSTRUCTION = decode_to_execute_INSTRUCTION;
  assign execute_MEMORY_STORE = decode_to_execute_MEMORY_STORE;
  assign execute_MEMORY_ENABLE = decode_to_execute_MEMORY_ENABLE;
  assign execute_ALIGNEMENT_FAULT = 1'b0;
  assign decode_FLUSH_ALL = _zz_67_;
  always @ (*) begin
    IBusCachedPlugin_rsp_issueDetected = _zz_73_;
    if(_zz_165_)begin
      IBusCachedPlugin_rsp_issueDetected = 1'b1;
    end
  end

  always @ (*) begin
    _zz_73_ = 1'b0;
    if(_zz_166_)begin
      _zz_73_ = 1'b1;
    end
  end

  always @ (*) begin
    _zz_74_ = memory_FORMAL_PC_NEXT;
    if(BranchPlugin_jumpInterface_valid)begin
      _zz_74_ = BranchPlugin_jumpInterface_payload;
    end
  end

  always @ (*) begin
    _zz_75_ = decode_FORMAL_PC_NEXT;
    if(IBusCachedPlugin_redoBranch_valid)begin
      _zz_75_ = IBusCachedPlugin_redoBranch_payload;
    end
  end

  assign decode_PC = _zz_78_;
  assign decode_INSTRUCTION = _zz_77_;
  assign writeBack_PC = memory_to_writeBack_PC;
  assign writeBack_INSTRUCTION = memory_to_writeBack_INSTRUCTION;
  always @ (*) begin
    decode_arbitration_haltItself = 1'b0;
    case(_zz_142_)
      3'b000 : begin
      end
      3'b001 : begin
      end
      3'b010 : begin
        decode_arbitration_haltItself = 1'b1;
      end
      3'b011 : begin
      end
      3'b100 : begin
      end
      default : begin
      end
    endcase
  end

  always @ (*) begin
    decode_arbitration_haltByOther = 1'b0;
    if((decode_arbitration_isValid && (_zz_133_ || _zz_134_)))begin
      decode_arbitration_haltByOther = 1'b1;
    end
    if((CsrPlugin_interrupt_valid && CsrPlugin_allowInterrupts))begin
      decode_arbitration_haltByOther = decode_arbitration_isValid;
    end
    if(({(writeBack_arbitration_isValid && (writeBack_ENV_CTRL == `EnvCtrlEnum_defaultEncoding_XRET)),{(memory_arbitration_isValid && (memory_ENV_CTRL == `EnvCtrlEnum_defaultEncoding_XRET)),(execute_arbitration_isValid && (execute_ENV_CTRL == `EnvCtrlEnum_defaultEncoding_XRET))}} != (3'b000)))begin
      decode_arbitration_haltByOther = 1'b1;
    end
  end

  always @ (*) begin
    decode_arbitration_removeIt = 1'b0;
    if(decode_arbitration_isFlushed)begin
      decode_arbitration_removeIt = 1'b1;
    end
  end

  assign decode_arbitration_flushIt = 1'b0;
  always @ (*) begin
    decode_arbitration_flushNext = 1'b0;
    if(IBusCachedPlugin_redoBranch_valid)begin
      decode_arbitration_flushNext = 1'b1;
    end
  end

  always @ (*) begin
    execute_arbitration_haltItself = 1'b0;
    if(((((execute_arbitration_isValid && execute_MEMORY_ENABLE) && (! dBus_cmd_ready)) && (! execute_DBusSimplePlugin_skipCmd)) && (! _zz_95_)))begin
      execute_arbitration_haltItself = 1'b1;
    end
    if(_zz_163_)begin
      if(_zz_167_)begin
        if(! execute_LightShifterPlugin_done) begin
          execute_arbitration_haltItself = 1'b1;
        end
      end
    end
    if(_zz_164_)begin
      if(execute_CsrPlugin_blockedBySideEffects)begin
        execute_arbitration_haltItself = 1'b1;
      end
    end
  end

  always @ (*) begin
    execute_arbitration_haltByOther = 1'b0;
    if(_zz_168_)begin
      execute_arbitration_haltByOther = 1'b1;
    end
  end

  always @ (*) begin
    execute_arbitration_removeIt = 1'b0;
    if(CsrPlugin_selfException_valid)begin
      execute_arbitration_removeIt = 1'b1;
    end
    if(execute_arbitration_isFlushed)begin
      execute_arbitration_removeIt = 1'b1;
    end
  end

  always @ (*) begin
    execute_arbitration_flushIt = 1'b0;
    if(_zz_168_)begin
      if(_zz_169_)begin
        execute_arbitration_flushIt = 1'b1;
      end
    end
  end

  always @ (*) begin
    execute_arbitration_flushNext = 1'b0;
    if(CsrPlugin_selfException_valid)begin
      execute_arbitration_flushNext = 1'b1;
    end
    if(_zz_168_)begin
      if(_zz_169_)begin
        execute_arbitration_flushNext = 1'b1;
      end
    end
  end

  always @ (*) begin
    memory_arbitration_haltItself = 1'b0;
    if((((memory_arbitration_isValid && memory_MEMORY_ENABLE) && (! memory_MEMORY_STORE)) && ((! dBus_rsp_ready) || 1'b0)))begin
      memory_arbitration_haltItself = 1'b1;
    end
  end

  assign memory_arbitration_haltByOther = 1'b0;
  always @ (*) begin
    memory_arbitration_removeIt = 1'b0;
    if(memory_arbitration_isFlushed)begin
      memory_arbitration_removeIt = 1'b1;
    end
  end

  assign memory_arbitration_flushIt = 1'b0;
  always @ (*) begin
    memory_arbitration_flushNext = 1'b0;
    if(BranchPlugin_jumpInterface_valid)begin
      memory_arbitration_flushNext = 1'b1;
    end
  end

  assign writeBack_arbitration_haltItself = 1'b0;
  assign writeBack_arbitration_haltByOther = 1'b0;
  always @ (*) begin
    writeBack_arbitration_removeIt = 1'b0;
    if(writeBack_arbitration_isFlushed)begin
      writeBack_arbitration_removeIt = 1'b1;
    end
  end

  assign writeBack_arbitration_flushIt = 1'b0;
  always @ (*) begin
    writeBack_arbitration_flushNext = 1'b0;
    if(_zz_170_)begin
      writeBack_arbitration_flushNext = 1'b1;
    end
    if(_zz_171_)begin
      writeBack_arbitration_flushNext = 1'b1;
    end
  end

  assign lastStageInstruction = writeBack_INSTRUCTION;
  assign lastStagePc = writeBack_PC;
  assign lastStageIsValid = writeBack_arbitration_isValid;
  assign lastStageIsFiring = writeBack_arbitration_isFiring;
  always @ (*) begin
    IBusCachedPlugin_fetcherHalt = 1'b0;
    if(({CsrPlugin_exceptionPortCtrl_exceptionValids_writeBack,{CsrPlugin_exceptionPortCtrl_exceptionValids_memory,{CsrPlugin_exceptionPortCtrl_exceptionValids_execute,CsrPlugin_exceptionPortCtrl_exceptionValids_decode}}} != (4'b0000)))begin
      IBusCachedPlugin_fetcherHalt = 1'b1;
    end
    if(_zz_170_)begin
      IBusCachedPlugin_fetcherHalt = 1'b1;
    end
    if(_zz_171_)begin
      IBusCachedPlugin_fetcherHalt = 1'b1;
    end
    if(_zz_168_)begin
      if(_zz_169_)begin
        IBusCachedPlugin_fetcherHalt = 1'b1;
      end
    end
    if(DebugPlugin_haltIt)begin
      IBusCachedPlugin_fetcherHalt = 1'b1;
    end
    if(_zz_172_)begin
      IBusCachedPlugin_fetcherHalt = 1'b1;
    end
  end

  always @ (*) begin
    IBusCachedPlugin_fetcherflushIt = 1'b0;
    if(({writeBack_arbitration_flushNext,{memory_arbitration_flushNext,{execute_arbitration_flushNext,decode_arbitration_flushNext}}} != (4'b0000)))begin
      IBusCachedPlugin_fetcherflushIt = 1'b1;
    end
    if(_zz_168_)begin
      if(_zz_169_)begin
        IBusCachedPlugin_fetcherflushIt = 1'b1;
      end
    end
  end

  always @ (*) begin
    IBusCachedPlugin_incomingInstruction = 1'b0;
    if(IBusCachedPlugin_iBusRsp_stages_1_input_valid)begin
      IBusCachedPlugin_incomingInstruction = 1'b1;
    end
    if(IBusCachedPlugin_injector_decodeInput_valid)begin
      IBusCachedPlugin_incomingInstruction = 1'b1;
    end
  end

  always @ (*) begin
    CsrPlugin_jumpInterface_valid = 1'b0;
    if(_zz_170_)begin
      CsrPlugin_jumpInterface_valid = 1'b1;
    end
    if(_zz_171_)begin
      CsrPlugin_jumpInterface_valid = 1'b1;
    end
  end

  always @ (*) begin
    CsrPlugin_jumpInterface_payload = (32'bxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx);
    if(_zz_170_)begin
      CsrPlugin_jumpInterface_payload = {CsrPlugin_xtvec_base,(2'b00)};
    end
    if(_zz_171_)begin
      case(_zz_173_)
        2'b11 : begin
          CsrPlugin_jumpInterface_payload = CsrPlugin_mepc;
        end
        default : begin
        end
      endcase
    end
  end

  always @ (*) begin
    CsrPlugin_forceMachineWire = 1'b0;
    if(DebugPlugin_godmode)begin
      CsrPlugin_forceMachineWire = 1'b1;
    end
  end

  always @ (*) begin
    CsrPlugin_allowInterrupts = 1'b1;
    if((DebugPlugin_haltIt || DebugPlugin_stepIt))begin
      CsrPlugin_allowInterrupts = 1'b0;
    end
  end

  always @ (*) begin
    CsrPlugin_allowException = 1'b1;
    if(DebugPlugin_godmode)begin
      CsrPlugin_allowException = 1'b0;
    end
  end

  assign IBusCachedPlugin_jump_pcLoad_valid = ({CsrPlugin_jumpInterface_valid,{BranchPlugin_jumpInterface_valid,IBusCachedPlugin_redoBranch_valid}} != (3'b000));
  assign _zz_80_ = {IBusCachedPlugin_redoBranch_valid,{BranchPlugin_jumpInterface_valid,CsrPlugin_jumpInterface_valid}};
  assign _zz_81_ = (_zz_80_ & (~ _zz_189_));
  assign _zz_82_ = _zz_81_[1];
  assign _zz_83_ = _zz_81_[2];
  assign IBusCachedPlugin_jump_pcLoad_payload = _zz_162_;
  always @ (*) begin
    IBusCachedPlugin_fetchPc_corrected = 1'b0;
    if(IBusCachedPlugin_jump_pcLoad_valid)begin
      IBusCachedPlugin_fetchPc_corrected = 1'b1;
    end
  end

  always @ (*) begin
    IBusCachedPlugin_fetchPc_pcRegPropagate = 1'b0;
    if(IBusCachedPlugin_iBusRsp_stages_1_input_ready)begin
      IBusCachedPlugin_fetchPc_pcRegPropagate = 1'b1;
    end
  end

  always @ (*) begin
    IBusCachedPlugin_fetchPc_pc = (IBusCachedPlugin_fetchPc_pcReg + _zz_191_);
    if(IBusCachedPlugin_jump_pcLoad_valid)begin
      IBusCachedPlugin_fetchPc_pc = IBusCachedPlugin_jump_pcLoad_payload;
    end
    IBusCachedPlugin_fetchPc_pc[0] = 1'b0;
    IBusCachedPlugin_fetchPc_pc[1] = 1'b0;
  end

  assign IBusCachedPlugin_fetchPc_output_valid = ((! IBusCachedPlugin_fetcherHalt) && IBusCachedPlugin_fetchPc_booted);
  assign IBusCachedPlugin_fetchPc_output_payload = IBusCachedPlugin_fetchPc_pc;
  assign IBusCachedPlugin_iBusRsp_stages_0_input_valid = IBusCachedPlugin_fetchPc_output_valid;
  assign IBusCachedPlugin_fetchPc_output_ready = IBusCachedPlugin_iBusRsp_stages_0_input_ready;
  assign IBusCachedPlugin_iBusRsp_stages_0_input_payload = IBusCachedPlugin_fetchPc_output_payload;
  assign IBusCachedPlugin_iBusRsp_stages_0_inputSample = 1'b1;
  always @ (*) begin
    IBusCachedPlugin_iBusRsp_stages_0_halt = 1'b0;
    if(IBusCachedPlugin_cache_io_cpu_prefetch_haltIt)begin
      IBusCachedPlugin_iBusRsp_stages_0_halt = 1'b1;
    end
  end

  assign _zz_84_ = (! IBusCachedPlugin_iBusRsp_stages_0_halt);
  assign IBusCachedPlugin_iBusRsp_stages_0_input_ready = (IBusCachedPlugin_iBusRsp_stages_0_output_ready && _zz_84_);
  assign IBusCachedPlugin_iBusRsp_stages_0_output_valid = (IBusCachedPlugin_iBusRsp_stages_0_input_valid && _zz_84_);
  assign IBusCachedPlugin_iBusRsp_stages_0_output_payload = IBusCachedPlugin_iBusRsp_stages_0_input_payload;
  always @ (*) begin
    IBusCachedPlugin_iBusRsp_stages_1_halt = 1'b0;
    if(IBusCachedPlugin_cache_io_cpu_fetch_haltIt)begin
      IBusCachedPlugin_iBusRsp_stages_1_halt = 1'b1;
    end
    if((IBusCachedPlugin_rsp_issueDetected || IBusCachedPlugin_rsp_iBusRspOutputHalt))begin
      IBusCachedPlugin_iBusRsp_stages_1_halt = 1'b1;
    end
  end

  assign _zz_85_ = (! IBusCachedPlugin_iBusRsp_stages_1_halt);
  assign IBusCachedPlugin_iBusRsp_stages_1_input_ready = (IBusCachedPlugin_iBusRsp_stages_1_output_ready && _zz_85_);
  assign IBusCachedPlugin_iBusRsp_stages_1_output_valid = (IBusCachedPlugin_iBusRsp_stages_1_input_valid && _zz_85_);
  assign IBusCachedPlugin_iBusRsp_stages_1_output_payload = IBusCachedPlugin_iBusRsp_stages_1_input_payload;
  assign IBusCachedPlugin_iBusRsp_stages_0_output_ready = _zz_86_;
  assign _zz_86_ = ((1'b0 && (! _zz_87_)) || IBusCachedPlugin_iBusRsp_stages_1_input_ready);
  assign _zz_87_ = _zz_88_;
  assign IBusCachedPlugin_iBusRsp_stages_1_input_valid = _zz_87_;
  assign IBusCachedPlugin_iBusRsp_stages_1_input_payload = IBusCachedPlugin_fetchPc_pcReg;
  always @ (*) begin
    IBusCachedPlugin_iBusRsp_readyForError = 1'b1;
    if(IBusCachedPlugin_injector_decodeInput_valid)begin
      IBusCachedPlugin_iBusRsp_readyForError = 1'b0;
    end
    if((! IBusCachedPlugin_pcValids_0))begin
      IBusCachedPlugin_iBusRsp_readyForError = 1'b0;
    end
  end

  assign IBusCachedPlugin_iBusRsp_output_ready = ((1'b0 && (! IBusCachedPlugin_injector_decodeInput_valid)) || IBusCachedPlugin_injector_decodeInput_ready);
  assign IBusCachedPlugin_injector_decodeInput_valid = _zz_89_;
  assign IBusCachedPlugin_injector_decodeInput_payload_pc = _zz_90_;
  assign IBusCachedPlugin_injector_decodeInput_payload_rsp_error = _zz_91_;
  assign IBusCachedPlugin_injector_decodeInput_payload_rsp_inst = _zz_92_;
  assign IBusCachedPlugin_injector_decodeInput_payload_isRvc = _zz_93_;
  assign _zz_79_ = (decode_arbitration_isStuck ? decode_INSTRUCTION : IBusCachedPlugin_iBusRsp_output_payload_rsp_inst);
  assign IBusCachedPlugin_pcValids_0 = IBusCachedPlugin_injector_nextPcCalc_valids_1;
  assign IBusCachedPlugin_pcValids_1 = IBusCachedPlugin_injector_nextPcCalc_valids_2;
  assign IBusCachedPlugin_pcValids_2 = IBusCachedPlugin_injector_nextPcCalc_valids_3;
  assign IBusCachedPlugin_pcValids_3 = IBusCachedPlugin_injector_nextPcCalc_valids_4;
  assign IBusCachedPlugin_injector_decodeInput_ready = (! decode_arbitration_isStuck);
  always @ (*) begin
    decode_arbitration_isValid = (IBusCachedPlugin_injector_decodeInput_valid && (! IBusCachedPlugin_injector_decodeRemoved));
    case(_zz_142_)
      3'b000 : begin
      end
      3'b001 : begin
      end
      3'b010 : begin
        decode_arbitration_isValid = 1'b1;
      end
      3'b011 : begin
        decode_arbitration_isValid = 1'b1;
      end
      3'b100 : begin
      end
      default : begin
      end
    endcase
  end

  assign _zz_78_ = IBusCachedPlugin_injector_decodeInput_payload_pc;
  assign _zz_77_ = IBusCachedPlugin_injector_decodeInput_payload_rsp_inst;
  assign _zz_76_ = (decode_PC + (32'b00000000000000000000000000000100));
  assign iBus_cmd_valid = IBusCachedPlugin_cache_io_mem_cmd_valid;
  always @ (*) begin
    iBus_cmd_payload_address = IBusCachedPlugin_cache_io_mem_cmd_payload_address;
    iBus_cmd_payload_address = IBusCachedPlugin_cache_io_mem_cmd_payload_address;
  end

  assign iBus_cmd_payload_size = IBusCachedPlugin_cache_io_mem_cmd_payload_size;
  assign IBusCachedPlugin_s0_tightlyCoupledHit = 1'b0;
  assign _zz_144_ = (IBusCachedPlugin_iBusRsp_stages_0_input_valid && (! IBusCachedPlugin_s0_tightlyCoupledHit));
  assign _zz_147_ = (32'b00000000000000000000000000000000);
  assign _zz_145_ = (IBusCachedPlugin_iBusRsp_stages_1_input_valid && (! IBusCachedPlugin_s1_tightlyCoupledHit));
  assign _zz_146_ = (! IBusCachedPlugin_iBusRsp_stages_1_input_ready);
  assign _zz_155_ = (CsrPlugin_privilege == (2'b00));
  assign IBusCachedPlugin_rsp_iBusRspOutputHalt = 1'b0;
  always @ (*) begin
    IBusCachedPlugin_rsp_redoFetch = 1'b0;
    if(_zz_166_)begin
      IBusCachedPlugin_rsp_redoFetch = 1'b1;
    end
    if(_zz_165_)begin
      IBusCachedPlugin_rsp_redoFetch = 1'b1;
    end
    if(_zz_174_)begin
      IBusCachedPlugin_rsp_redoFetch = 1'b0;
    end
  end

  always @ (*) begin
    _zz_159_ = (IBusCachedPlugin_rsp_redoFetch && (! IBusCachedPlugin_cache_io_cpu_fetch_mmuRefilling));
    if(_zz_165_)begin
      _zz_159_ = 1'b1;
    end
    if(_zz_174_)begin
      _zz_159_ = 1'b0;
    end
  end

  assign IBusCachedPlugin_redoBranch_valid = IBusCachedPlugin_rsp_redoFetch;
  assign IBusCachedPlugin_redoBranch_payload = IBusCachedPlugin_iBusRsp_stages_1_input_payload;
  assign IBusCachedPlugin_iBusRsp_output_valid = IBusCachedPlugin_iBusRsp_stages_1_output_valid;
  assign IBusCachedPlugin_iBusRsp_stages_1_output_ready = IBusCachedPlugin_iBusRsp_output_ready;
  assign IBusCachedPlugin_iBusRsp_output_payload_rsp_inst = IBusCachedPlugin_cache_io_cpu_fetch_data;
  assign IBusCachedPlugin_iBusRsp_output_payload_pc = IBusCachedPlugin_iBusRsp_stages_1_output_payload;
  assign _zz_151_ = 1'b1;
  assign _zz_149_ = 1'b1;
  assign _zz_150_ = 1'b1;
  assign _zz_148_ = 1'b0;
  assign _zz_152_ = 1'b0;
  assign _zz_153_ = 1'b0;
  assign _zz_154_ = 1'b0;
  assign _zz_143_ = (decode_arbitration_isValid && decode_FLUSH_ALL);
  assign _zz_95_ = 1'b0;
  always @ (*) begin
    execute_DBusSimplePlugin_skipCmd = 1'b0;
    if(execute_ALIGNEMENT_FAULT)begin
      execute_DBusSimplePlugin_skipCmd = 1'b1;
    end
  end

  assign dBus_cmd_valid = (((((execute_arbitration_isValid && execute_MEMORY_ENABLE) && (! execute_arbitration_isStuckByOthers)) && (! execute_arbitration_isFlushed)) && (! execute_DBusSimplePlugin_skipCmd)) && (! _zz_95_));
  assign dBus_cmd_payload_wr = execute_MEMORY_STORE;
  assign dBus_cmd_payload_size = execute_INSTRUCTION[13 : 12];
  always @ (*) begin
    case(dBus_cmd_payload_size)
      2'b00 : begin
        _zz_96_ = {{{execute_RS2[7 : 0],execute_RS2[7 : 0]},execute_RS2[7 : 0]},execute_RS2[7 : 0]};
      end
      2'b01 : begin
        _zz_96_ = {execute_RS2[15 : 0],execute_RS2[15 : 0]};
      end
      default : begin
        _zz_96_ = execute_RS2[31 : 0];
      end
    endcase
  end

  assign dBus_cmd_payload_data = _zz_96_;
  assign _zz_72_ = dBus_cmd_payload_address[1 : 0];
  always @ (*) begin
    case(dBus_cmd_payload_size)
      2'b00 : begin
        _zz_97_ = (4'b0001);
      end
      2'b01 : begin
        _zz_97_ = (4'b0011);
      end
      default : begin
        _zz_97_ = (4'b1111);
      end
    endcase
  end

  assign execute_DBusSimplePlugin_formalMask = (_zz_97_ <<< dBus_cmd_payload_address[1 : 0]);
  assign dBus_cmd_payload_address = execute_SRC_ADD;
  assign _zz_71_ = dBus_rsp_data;
  always @ (*) begin
    writeBack_DBusSimplePlugin_rspShifted = writeBack_MEMORY_READ_DATA;
    case(writeBack_MEMORY_ADDRESS_LOW)
      2'b01 : begin
        writeBack_DBusSimplePlugin_rspShifted[7 : 0] = writeBack_MEMORY_READ_DATA[15 : 8];
      end
      2'b10 : begin
        writeBack_DBusSimplePlugin_rspShifted[15 : 0] = writeBack_MEMORY_READ_DATA[31 : 16];
      end
      2'b11 : begin
        writeBack_DBusSimplePlugin_rspShifted[7 : 0] = writeBack_MEMORY_READ_DATA[31 : 24];
      end
      default : begin
      end
    endcase
  end

  assign _zz_98_ = (writeBack_DBusSimplePlugin_rspShifted[7] && (! writeBack_INSTRUCTION[14]));
  always @ (*) begin
    _zz_99_[31] = _zz_98_;
    _zz_99_[30] = _zz_98_;
    _zz_99_[29] = _zz_98_;
    _zz_99_[28] = _zz_98_;
    _zz_99_[27] = _zz_98_;
    _zz_99_[26] = _zz_98_;
    _zz_99_[25] = _zz_98_;
    _zz_99_[24] = _zz_98_;
    _zz_99_[23] = _zz_98_;
    _zz_99_[22] = _zz_98_;
    _zz_99_[21] = _zz_98_;
    _zz_99_[20] = _zz_98_;
    _zz_99_[19] = _zz_98_;
    _zz_99_[18] = _zz_98_;
    _zz_99_[17] = _zz_98_;
    _zz_99_[16] = _zz_98_;
    _zz_99_[15] = _zz_98_;
    _zz_99_[14] = _zz_98_;
    _zz_99_[13] = _zz_98_;
    _zz_99_[12] = _zz_98_;
    _zz_99_[11] = _zz_98_;
    _zz_99_[10] = _zz_98_;
    _zz_99_[9] = _zz_98_;
    _zz_99_[8] = _zz_98_;
    _zz_99_[7 : 0] = writeBack_DBusSimplePlugin_rspShifted[7 : 0];
  end

  assign _zz_100_ = (writeBack_DBusSimplePlugin_rspShifted[15] && (! writeBack_INSTRUCTION[14]));
  always @ (*) begin
    _zz_101_[31] = _zz_100_;
    _zz_101_[30] = _zz_100_;
    _zz_101_[29] = _zz_100_;
    _zz_101_[28] = _zz_100_;
    _zz_101_[27] = _zz_100_;
    _zz_101_[26] = _zz_100_;
    _zz_101_[25] = _zz_100_;
    _zz_101_[24] = _zz_100_;
    _zz_101_[23] = _zz_100_;
    _zz_101_[22] = _zz_100_;
    _zz_101_[21] = _zz_100_;
    _zz_101_[20] = _zz_100_;
    _zz_101_[19] = _zz_100_;
    _zz_101_[18] = _zz_100_;
    _zz_101_[17] = _zz_100_;
    _zz_101_[16] = _zz_100_;
    _zz_101_[15 : 0] = writeBack_DBusSimplePlugin_rspShifted[15 : 0];
  end

  always @ (*) begin
    case(_zz_187_)
      2'b00 : begin
        writeBack_DBusSimplePlugin_rspFormated = _zz_99_;
      end
      2'b01 : begin
        writeBack_DBusSimplePlugin_rspFormated = _zz_101_;
      end
      default : begin
        writeBack_DBusSimplePlugin_rspFormated = writeBack_DBusSimplePlugin_rspShifted;
      end
    endcase
  end

  assign _zz_103_ = ((decode_INSTRUCTION & (32'b00000000000000000110000000000100)) == (32'b00000000000000000010000000000000));
  assign _zz_104_ = ((decode_INSTRUCTION & (32'b00000000000000000000000000000100)) == (32'b00000000000000000000000000000100));
  assign _zz_105_ = ((decode_INSTRUCTION & (32'b00000000000000000000000001001000)) == (32'b00000000000000000000000001001000));
  assign _zz_106_ = ((decode_INSTRUCTION & (32'b00000000000000000000000001010000)) == (32'b00000000000000000000000000010000));
  assign _zz_107_ = ((decode_INSTRUCTION & (32'b00000000000000000100000001010000)) == (32'b00000000000000000100000001010000));
  assign _zz_102_ = {({(_zz_239_ == _zz_240_),(_zz_241_ == _zz_242_)} != (2'b00)),{({_zz_243_,_zz_244_} != (2'b00)),{(_zz_245_ != (1'b0)),{(_zz_246_ != _zz_247_),{_zz_248_,{_zz_249_,_zz_250_}}}}}};
  assign _zz_69_ = _zz_192_[0];
  assign _zz_68_ = _zz_193_[0];
  assign _zz_67_ = _zz_194_[0];
  assign _zz_108_ = _zz_102_[4 : 3];
  assign _zz_66_ = _zz_108_;
  assign _zz_109_ = _zz_102_[6 : 5];
  assign _zz_65_ = _zz_109_;
  assign _zz_64_ = _zz_195_[0];
  assign _zz_110_ = _zz_102_[10 : 9];
  assign _zz_63_ = _zz_110_;
  assign _zz_62_ = _zz_196_[0];
  assign _zz_111_ = _zz_102_[13 : 12];
  assign _zz_61_ = _zz_111_;
  assign _zz_60_ = _zz_197_[0];
  assign _zz_59_ = _zz_198_[0];
  assign _zz_112_ = _zz_102_[17 : 16];
  assign _zz_58_ = _zz_112_;
  assign _zz_113_ = _zz_102_[19 : 18];
  assign _zz_57_ = _zz_113_;
  assign _zz_56_ = _zz_199_[0];
  assign _zz_55_ = _zz_200_[0];
  assign _zz_54_ = _zz_201_[0];
  assign _zz_53_ = _zz_202_[0];
  assign _zz_114_ = _zz_102_[25 : 24];
  assign _zz_52_ = _zz_114_;
  assign _zz_51_ = _zz_203_[0];
  assign _zz_50_ = _zz_204_[0];
  assign decode_RegFilePlugin_regFileReadAddress1 = decode_INSTRUCTION_ANTICIPATED[19 : 15];
  assign decode_RegFilePlugin_regFileReadAddress2 = decode_INSTRUCTION_ANTICIPATED[24 : 20];
  assign decode_RegFilePlugin_rs1Data = _zz_160_;
  assign decode_RegFilePlugin_rs2Data = _zz_161_;
  assign _zz_49_ = decode_RegFilePlugin_rs1Data;
  assign _zz_48_ = decode_RegFilePlugin_rs2Data;
  assign lastStageRegFileWrite_valid = (_zz_46_ && writeBack_arbitration_isFiring);
  assign lastStageRegFileWrite_payload_address = _zz_45_[11 : 7];
  assign lastStageRegFileWrite_payload_data = _zz_70_;
  always @ (*) begin
    case(execute_ALU_BITWISE_CTRL)
      `AluBitwiseCtrlEnum_defaultEncoding_AND_1 : begin
        execute_IntAluPlugin_bitwise = (execute_SRC1 & execute_SRC2);
      end
      `AluBitwiseCtrlEnum_defaultEncoding_OR_1 : begin
        execute_IntAluPlugin_bitwise = (execute_SRC1 | execute_SRC2);
      end
      default : begin
        execute_IntAluPlugin_bitwise = (execute_SRC1 ^ execute_SRC2);
      end
    endcase
  end

  always @ (*) begin
    case(execute_ALU_CTRL)
      `AluCtrlEnum_defaultEncoding_BITWISE : begin
        _zz_115_ = execute_IntAluPlugin_bitwise;
      end
      `AluCtrlEnum_defaultEncoding_SLT_SLTU : begin
        _zz_115_ = {31'd0, _zz_205_};
      end
      default : begin
        _zz_115_ = execute_SRC_ADD_SUB;
      end
    endcase
  end

  assign _zz_43_ = _zz_115_;
  assign _zz_41_ = (decode_SRC_ADD_ZERO && (! decode_SRC_USE_SUB_LESS));
  always @ (*) begin
    case(decode_SRC1_CTRL)
      `Src1CtrlEnum_defaultEncoding_RS : begin
        _zz_116_ = _zz_38_;
      end
      `Src1CtrlEnum_defaultEncoding_PC_INCREMENT : begin
        _zz_116_ = {29'd0, _zz_206_};
      end
      `Src1CtrlEnum_defaultEncoding_IMU : begin
        _zz_116_ = {decode_INSTRUCTION[31 : 12],(12'b000000000000)};
      end
      default : begin
        _zz_116_ = {27'd0, _zz_207_};
      end
    endcase
  end

  assign _zz_40_ = _zz_116_;
  assign _zz_117_ = _zz_208_[11];
  always @ (*) begin
    _zz_118_[19] = _zz_117_;
    _zz_118_[18] = _zz_117_;
    _zz_118_[17] = _zz_117_;
    _zz_118_[16] = _zz_117_;
    _zz_118_[15] = _zz_117_;
    _zz_118_[14] = _zz_117_;
    _zz_118_[13] = _zz_117_;
    _zz_118_[12] = _zz_117_;
    _zz_118_[11] = _zz_117_;
    _zz_118_[10] = _zz_117_;
    _zz_118_[9] = _zz_117_;
    _zz_118_[8] = _zz_117_;
    _zz_118_[7] = _zz_117_;
    _zz_118_[6] = _zz_117_;
    _zz_118_[5] = _zz_117_;
    _zz_118_[4] = _zz_117_;
    _zz_118_[3] = _zz_117_;
    _zz_118_[2] = _zz_117_;
    _zz_118_[1] = _zz_117_;
    _zz_118_[0] = _zz_117_;
  end

  assign _zz_119_ = _zz_209_[11];
  always @ (*) begin
    _zz_120_[19] = _zz_119_;
    _zz_120_[18] = _zz_119_;
    _zz_120_[17] = _zz_119_;
    _zz_120_[16] = _zz_119_;
    _zz_120_[15] = _zz_119_;
    _zz_120_[14] = _zz_119_;
    _zz_120_[13] = _zz_119_;
    _zz_120_[12] = _zz_119_;
    _zz_120_[11] = _zz_119_;
    _zz_120_[10] = _zz_119_;
    _zz_120_[9] = _zz_119_;
    _zz_120_[8] = _zz_119_;
    _zz_120_[7] = _zz_119_;
    _zz_120_[6] = _zz_119_;
    _zz_120_[5] = _zz_119_;
    _zz_120_[4] = _zz_119_;
    _zz_120_[3] = _zz_119_;
    _zz_120_[2] = _zz_119_;
    _zz_120_[1] = _zz_119_;
    _zz_120_[0] = _zz_119_;
  end

  always @ (*) begin
    case(decode_SRC2_CTRL)
      `Src2CtrlEnum_defaultEncoding_RS : begin
        _zz_121_ = _zz_35_;
      end
      `Src2CtrlEnum_defaultEncoding_IMI : begin
        _zz_121_ = {_zz_118_,decode_INSTRUCTION[31 : 20]};
      end
      `Src2CtrlEnum_defaultEncoding_IMS : begin
        _zz_121_ = {_zz_120_,{decode_INSTRUCTION[31 : 25],decode_INSTRUCTION[11 : 7]}};
      end
      default : begin
        _zz_121_ = _zz_34_;
      end
    endcase
  end

  assign _zz_37_ = _zz_121_;
  always @ (*) begin
    execute_SrcPlugin_addSub = _zz_210_;
    if(execute_SRC2_FORCE_ZERO)begin
      execute_SrcPlugin_addSub = execute_SRC1;
    end
  end

  assign execute_SrcPlugin_less = ((execute_SRC1[31] == execute_SRC2[31]) ? execute_SrcPlugin_addSub[31] : (execute_SRC_LESS_UNSIGNED ? execute_SRC2[31] : execute_SRC1[31]));
  assign _zz_33_ = execute_SrcPlugin_addSub;
  assign _zz_32_ = execute_SrcPlugin_addSub;
  assign _zz_31_ = execute_SrcPlugin_less;
  assign execute_LightShifterPlugin_isShift = (execute_SHIFT_CTRL != `ShiftCtrlEnum_defaultEncoding_DISABLE_1);
  assign execute_LightShifterPlugin_amplitude = (execute_LightShifterPlugin_isActive ? execute_LightShifterPlugin_amplitudeReg : execute_SRC2[4 : 0]);
  assign execute_LightShifterPlugin_shiftInput = (execute_LightShifterPlugin_isActive ? memory_REGFILE_WRITE_DATA : execute_SRC1);
  assign execute_LightShifterPlugin_done = (execute_LightShifterPlugin_amplitude[4 : 1] == (4'b0000));
  always @ (*) begin
    case(execute_SHIFT_CTRL)
      `ShiftCtrlEnum_defaultEncoding_SLL_1 : begin
        _zz_122_ = (execute_LightShifterPlugin_shiftInput <<< 1);
      end
      default : begin
        _zz_122_ = _zz_217_;
      end
    endcase
  end

  assign execute_BranchPlugin_eq = (execute_SRC1 == execute_SRC2);
  assign _zz_123_ = execute_INSTRUCTION[14 : 12];
  always @ (*) begin
    if((_zz_123_ == (3'b000))) begin
        _zz_124_ = execute_BranchPlugin_eq;
    end else if((_zz_123_ == (3'b001))) begin
        _zz_124_ = (! execute_BranchPlugin_eq);
    end else if((((_zz_123_ & (3'b101)) == (3'b101)))) begin
        _zz_124_ = (! execute_SRC_LESS);
    end else begin
        _zz_124_ = execute_SRC_LESS;
    end
  end

  always @ (*) begin
    case(execute_BRANCH_CTRL)
      `BranchCtrlEnum_defaultEncoding_INC : begin
        _zz_125_ = 1'b0;
      end
      `BranchCtrlEnum_defaultEncoding_JAL : begin
        _zz_125_ = 1'b1;
      end
      `BranchCtrlEnum_defaultEncoding_JALR : begin
        _zz_125_ = 1'b1;
      end
      default : begin
        _zz_125_ = _zz_124_;
      end
    endcase
  end

  assign _zz_28_ = _zz_125_;
  assign execute_BranchPlugin_branch_src1 = ((execute_BRANCH_CTRL == `BranchCtrlEnum_defaultEncoding_JALR) ? execute_RS1 : execute_PC);
  assign _zz_126_ = _zz_219_[19];
  always @ (*) begin
    _zz_127_[10] = _zz_126_;
    _zz_127_[9] = _zz_126_;
    _zz_127_[8] = _zz_126_;
    _zz_127_[7] = _zz_126_;
    _zz_127_[6] = _zz_126_;
    _zz_127_[5] = _zz_126_;
    _zz_127_[4] = _zz_126_;
    _zz_127_[3] = _zz_126_;
    _zz_127_[2] = _zz_126_;
    _zz_127_[1] = _zz_126_;
    _zz_127_[0] = _zz_126_;
  end

  assign _zz_128_ = _zz_220_[11];
  always @ (*) begin
    _zz_129_[19] = _zz_128_;
    _zz_129_[18] = _zz_128_;
    _zz_129_[17] = _zz_128_;
    _zz_129_[16] = _zz_128_;
    _zz_129_[15] = _zz_128_;
    _zz_129_[14] = _zz_128_;
    _zz_129_[13] = _zz_128_;
    _zz_129_[12] = _zz_128_;
    _zz_129_[11] = _zz_128_;
    _zz_129_[10] = _zz_128_;
    _zz_129_[9] = _zz_128_;
    _zz_129_[8] = _zz_128_;
    _zz_129_[7] = _zz_128_;
    _zz_129_[6] = _zz_128_;
    _zz_129_[5] = _zz_128_;
    _zz_129_[4] = _zz_128_;
    _zz_129_[3] = _zz_128_;
    _zz_129_[2] = _zz_128_;
    _zz_129_[1] = _zz_128_;
    _zz_129_[0] = _zz_128_;
  end

  assign _zz_130_ = _zz_221_[11];
  always @ (*) begin
    _zz_131_[18] = _zz_130_;
    _zz_131_[17] = _zz_130_;
    _zz_131_[16] = _zz_130_;
    _zz_131_[15] = _zz_130_;
    _zz_131_[14] = _zz_130_;
    _zz_131_[13] = _zz_130_;
    _zz_131_[12] = _zz_130_;
    _zz_131_[11] = _zz_130_;
    _zz_131_[10] = _zz_130_;
    _zz_131_[9] = _zz_130_;
    _zz_131_[8] = _zz_130_;
    _zz_131_[7] = _zz_130_;
    _zz_131_[6] = _zz_130_;
    _zz_131_[5] = _zz_130_;
    _zz_131_[4] = _zz_130_;
    _zz_131_[3] = _zz_130_;
    _zz_131_[2] = _zz_130_;
    _zz_131_[1] = _zz_130_;
    _zz_131_[0] = _zz_130_;
  end

  always @ (*) begin
    case(execute_BRANCH_CTRL)
      `BranchCtrlEnum_defaultEncoding_JAL : begin
        _zz_132_ = {{_zz_127_,{{{execute_INSTRUCTION[31],execute_INSTRUCTION[19 : 12]},execute_INSTRUCTION[20]},execute_INSTRUCTION[30 : 21]}},1'b0};
      end
      `BranchCtrlEnum_defaultEncoding_JALR : begin
        _zz_132_ = {_zz_129_,execute_INSTRUCTION[31 : 20]};
      end
      default : begin
        _zz_132_ = {{_zz_131_,{{{execute_INSTRUCTION[31],execute_INSTRUCTION[7]},execute_INSTRUCTION[30 : 25]},execute_INSTRUCTION[11 : 8]}},1'b0};
      end
    endcase
  end

  assign execute_BranchPlugin_branch_src2 = _zz_132_;
  assign execute_BranchPlugin_branchAdder = (execute_BranchPlugin_branch_src1 + execute_BranchPlugin_branch_src2);
  assign _zz_26_ = {execute_BranchPlugin_branchAdder[31 : 1],(1'b0)};
  assign BranchPlugin_jumpInterface_valid = ((memory_arbitration_isValid && memory_BRANCH_DO) && (! 1'b0));
  assign BranchPlugin_jumpInterface_payload = memory_BRANCH_CALC;
  always @ (*) begin
    _zz_133_ = 1'b0;
    if(_zz_136_)begin
      if((_zz_137_ == decode_INSTRUCTION[19 : 15]))begin
        _zz_133_ = 1'b1;
      end
    end
    if(_zz_175_)begin
      if(_zz_176_)begin
        if((writeBack_INSTRUCTION[11 : 7] == decode_INSTRUCTION[19 : 15]))begin
          _zz_133_ = 1'b1;
        end
      end
    end
    if(_zz_177_)begin
      if(_zz_178_)begin
        if((memory_INSTRUCTION[11 : 7] == decode_INSTRUCTION[19 : 15]))begin
          _zz_133_ = 1'b1;
        end
      end
    end
    if(_zz_179_)begin
      if(_zz_180_)begin
        if((execute_INSTRUCTION[11 : 7] == decode_INSTRUCTION[19 : 15]))begin
          _zz_133_ = 1'b1;
        end
      end
    end
    if((! decode_RS1_USE))begin
      _zz_133_ = 1'b0;
    end
  end

  always @ (*) begin
    _zz_134_ = 1'b0;
    if(_zz_136_)begin
      if((_zz_137_ == decode_INSTRUCTION[24 : 20]))begin
        _zz_134_ = 1'b1;
      end
    end
    if(_zz_175_)begin
      if(_zz_176_)begin
        if((writeBack_INSTRUCTION[11 : 7] == decode_INSTRUCTION[24 : 20]))begin
          _zz_134_ = 1'b1;
        end
      end
    end
    if(_zz_177_)begin
      if(_zz_178_)begin
        if((memory_INSTRUCTION[11 : 7] == decode_INSTRUCTION[24 : 20]))begin
          _zz_134_ = 1'b1;
        end
      end
    end
    if(_zz_179_)begin
      if(_zz_180_)begin
        if((execute_INSTRUCTION[11 : 7] == decode_INSTRUCTION[24 : 20]))begin
          _zz_134_ = 1'b1;
        end
      end
    end
    if((! decode_RS2_USE))begin
      _zz_134_ = 1'b0;
    end
  end

  assign _zz_135_ = (_zz_46_ && writeBack_arbitration_isFiring);
  always @ (*) begin
    CsrPlugin_privilege = (2'b11);
    if(CsrPlugin_forceMachineWire)begin
      CsrPlugin_privilege = (2'b11);
    end
  end

  assign CsrPlugin_misa_base = (2'b01);
  assign CsrPlugin_misa_extensions = (26'b00000000000000000000000000);
  assign _zz_138_ = (CsrPlugin_mip_MTIP && CsrPlugin_mie_MTIE);
  assign _zz_139_ = (CsrPlugin_mip_MSIP && CsrPlugin_mie_MSIE);
  assign _zz_140_ = (CsrPlugin_mip_MEIP && CsrPlugin_mie_MEIE);
  assign CsrPlugin_exceptionPortCtrl_exceptionValidsRegs_decode = 1'b0;
  assign CsrPlugin_exceptionPortCtrl_exceptionTargetPrivilegeUncapped = (2'b11);
  assign CsrPlugin_exceptionPortCtrl_exceptionTargetPrivilege = ((CsrPlugin_privilege < CsrPlugin_exceptionPortCtrl_exceptionTargetPrivilegeUncapped) ? CsrPlugin_exceptionPortCtrl_exceptionTargetPrivilegeUncapped : CsrPlugin_privilege);
  assign CsrPlugin_exceptionPortCtrl_exceptionValids_decode = CsrPlugin_exceptionPortCtrl_exceptionValidsRegs_decode;
  always @ (*) begin
    CsrPlugin_exceptionPortCtrl_exceptionValids_execute = CsrPlugin_exceptionPortCtrl_exceptionValidsRegs_execute;
    if(CsrPlugin_selfException_valid)begin
      CsrPlugin_exceptionPortCtrl_exceptionValids_execute = 1'b1;
    end
    if(execute_arbitration_isFlushed)begin
      CsrPlugin_exceptionPortCtrl_exceptionValids_execute = 1'b0;
    end
  end

  always @ (*) begin
    CsrPlugin_exceptionPortCtrl_exceptionValids_memory = CsrPlugin_exceptionPortCtrl_exceptionValidsRegs_memory;
    if(memory_arbitration_isFlushed)begin
      CsrPlugin_exceptionPortCtrl_exceptionValids_memory = 1'b0;
    end
  end

  always @ (*) begin
    CsrPlugin_exceptionPortCtrl_exceptionValids_writeBack = CsrPlugin_exceptionPortCtrl_exceptionValidsRegs_writeBack;
    if(writeBack_arbitration_isFlushed)begin
      CsrPlugin_exceptionPortCtrl_exceptionValids_writeBack = 1'b0;
    end
  end

  assign CsrPlugin_exceptionPendings_0 = CsrPlugin_exceptionPortCtrl_exceptionValidsRegs_decode;
  assign CsrPlugin_exceptionPendings_1 = CsrPlugin_exceptionPortCtrl_exceptionValidsRegs_execute;
  assign CsrPlugin_exceptionPendings_2 = CsrPlugin_exceptionPortCtrl_exceptionValidsRegs_memory;
  assign CsrPlugin_exceptionPendings_3 = CsrPlugin_exceptionPortCtrl_exceptionValidsRegs_writeBack;
  assign CsrPlugin_exception = (CsrPlugin_exceptionPortCtrl_exceptionValids_writeBack && CsrPlugin_allowException);
  assign CsrPlugin_lastStageWasWfi = 1'b0;
  always @ (*) begin
    CsrPlugin_pipelineLiberator_done = ((! ({writeBack_arbitration_isValid,{memory_arbitration_isValid,execute_arbitration_isValid}} != (3'b000))) && IBusCachedPlugin_pcValids_3);
    if(({CsrPlugin_exceptionPortCtrl_exceptionValidsRegs_writeBack,{CsrPlugin_exceptionPortCtrl_exceptionValidsRegs_memory,CsrPlugin_exceptionPortCtrl_exceptionValidsRegs_execute}} != (3'b000)))begin
      CsrPlugin_pipelineLiberator_done = 1'b0;
    end
    if(CsrPlugin_hadException)begin
      CsrPlugin_pipelineLiberator_done = 1'b0;
    end
  end

  assign CsrPlugin_interruptJump = ((CsrPlugin_interrupt_valid && CsrPlugin_pipelineLiberator_done) && CsrPlugin_allowInterrupts);
  always @ (*) begin
    CsrPlugin_targetPrivilege = CsrPlugin_interrupt_targetPrivilege;
    if(CsrPlugin_hadException)begin
      CsrPlugin_targetPrivilege = CsrPlugin_exceptionPortCtrl_exceptionTargetPrivilege;
    end
  end

  always @ (*) begin
    CsrPlugin_trapCause = CsrPlugin_interrupt_code;
    if(CsrPlugin_hadException)begin
      CsrPlugin_trapCause = CsrPlugin_exceptionPortCtrl_exceptionContext_code;
    end
  end

  always @ (*) begin
    CsrPlugin_xtvec_mode = (2'bxx);
    case(CsrPlugin_targetPrivilege)
      2'b11 : begin
        CsrPlugin_xtvec_mode = CsrPlugin_mtvec_mode;
      end
      default : begin
      end
    endcase
  end

  always @ (*) begin
    CsrPlugin_xtvec_base = (30'bxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx);
    case(CsrPlugin_targetPrivilege)
      2'b11 : begin
        CsrPlugin_xtvec_base = CsrPlugin_mtvec_base;
      end
      default : begin
      end
    endcase
  end

  assign contextSwitching = CsrPlugin_jumpInterface_valid;
  assign _zz_24_ = (! (((decode_INSTRUCTION[14 : 13] == (2'b01)) && (decode_INSTRUCTION[19 : 15] == (5'b00000))) || ((decode_INSTRUCTION[14 : 13] == (2'b11)) && (decode_INSTRUCTION[19 : 15] == (5'b00000)))));
  assign _zz_23_ = (decode_INSTRUCTION[13 : 7] != (7'b0100000));
  assign execute_CsrPlugin_inWfi = 1'b0;
  assign execute_CsrPlugin_blockedBySideEffects = ({writeBack_arbitration_isValid,memory_arbitration_isValid} != (2'b00));
  always @ (*) begin
    execute_CsrPlugin_illegalAccess = 1'b1;
    case(execute_CsrPlugin_csrAddress)
      12'b001100000000 : begin
        execute_CsrPlugin_illegalAccess = 1'b0;
      end
      12'b001101000001 : begin
        execute_CsrPlugin_illegalAccess = 1'b0;
      end
      12'b001100000101 : begin
        if(execute_CSR_WRITE_OPCODE)begin
          execute_CsrPlugin_illegalAccess = 1'b0;
        end
      end
      12'b001101000100 : begin
        execute_CsrPlugin_illegalAccess = 1'b0;
      end
      12'b001100000100 : begin
        execute_CsrPlugin_illegalAccess = 1'b0;
      end
      12'b001101000010 : begin
        if(execute_CSR_READ_OPCODE)begin
          execute_CsrPlugin_illegalAccess = 1'b0;
        end
      end
      default : begin
      end
    endcase
    if((CsrPlugin_privilege < execute_CsrPlugin_csrAddress[9 : 8]))begin
      execute_CsrPlugin_illegalAccess = 1'b1;
    end
    if(((! execute_arbitration_isValid) || (! execute_IS_CSR)))begin
      execute_CsrPlugin_illegalAccess = 1'b0;
    end
  end

  always @ (*) begin
    execute_CsrPlugin_illegalInstruction = 1'b0;
    if((execute_arbitration_isValid && (execute_ENV_CTRL == `EnvCtrlEnum_defaultEncoding_XRET)))begin
      if((CsrPlugin_privilege < execute_INSTRUCTION[29 : 28]))begin
        execute_CsrPlugin_illegalInstruction = 1'b1;
      end
    end
  end

  always @ (*) begin
    CsrPlugin_selfException_valid = 1'b0;
    if(_zz_181_)begin
      CsrPlugin_selfException_valid = 1'b1;
    end
  end

  always @ (*) begin
    CsrPlugin_selfException_payload_code = (4'bxxxx);
    if(_zz_181_)begin
      case(CsrPlugin_privilege)
        2'b00 : begin
          CsrPlugin_selfException_payload_code = (4'b1000);
        end
        default : begin
          CsrPlugin_selfException_payload_code = (4'b1011);
        end
      endcase
    end
  end

  assign CsrPlugin_selfException_payload_badAddr = execute_INSTRUCTION;
  always @ (*) begin
    execute_CsrPlugin_readData = (32'b00000000000000000000000000000000);
    case(execute_CsrPlugin_csrAddress)
      12'b001100000000 : begin
        execute_CsrPlugin_readData[12 : 11] = CsrPlugin_mstatus_MPP;
        execute_CsrPlugin_readData[7 : 7] = CsrPlugin_mstatus_MPIE;
        execute_CsrPlugin_readData[3 : 3] = CsrPlugin_mstatus_MIE;
      end
      12'b001101000001 : begin
        execute_CsrPlugin_readData[31 : 0] = CsrPlugin_mepc;
      end
      12'b001100000101 : begin
      end
      12'b001101000100 : begin
        execute_CsrPlugin_readData[11 : 11] = CsrPlugin_mip_MEIP;
        execute_CsrPlugin_readData[7 : 7] = CsrPlugin_mip_MTIP;
        execute_CsrPlugin_readData[3 : 3] = CsrPlugin_mip_MSIP;
      end
      12'b001100000100 : begin
        execute_CsrPlugin_readData[11 : 11] = CsrPlugin_mie_MEIE;
        execute_CsrPlugin_readData[7 : 7] = CsrPlugin_mie_MTIE;
        execute_CsrPlugin_readData[3 : 3] = CsrPlugin_mie_MSIE;
      end
      12'b001101000010 : begin
        execute_CsrPlugin_readData[31 : 31] = CsrPlugin_mcause_interrupt;
        execute_CsrPlugin_readData[3 : 0] = CsrPlugin_mcause_exceptionCode;
      end
      default : begin
      end
    endcase
  end

  assign execute_CsrPlugin_writeInstruction = ((execute_arbitration_isValid && execute_IS_CSR) && execute_CSR_WRITE_OPCODE);
  assign execute_CsrPlugin_readInstruction = ((execute_arbitration_isValid && execute_IS_CSR) && execute_CSR_READ_OPCODE);
  assign execute_CsrPlugin_writeEnable = ((execute_CsrPlugin_writeInstruction && (! execute_CsrPlugin_blockedBySideEffects)) && (! execute_arbitration_isStuckByOthers));
  assign execute_CsrPlugin_readEnable = ((execute_CsrPlugin_readInstruction && (! execute_CsrPlugin_blockedBySideEffects)) && (! execute_arbitration_isStuckByOthers));
  assign execute_CsrPlugin_readToWriteData = execute_CsrPlugin_readData;
  always @ (*) begin
    case(_zz_188_)
      1'b0 : begin
        execute_CsrPlugin_writeData = execute_SRC1;
      end
      default : begin
        execute_CsrPlugin_writeData = (execute_INSTRUCTION[12] ? (execute_CsrPlugin_readToWriteData & (~ execute_SRC1)) : (execute_CsrPlugin_readToWriteData | execute_SRC1));
      end
    endcase
  end

  assign execute_CsrPlugin_csrAddress = execute_INSTRUCTION[31 : 20];
  always @ (*) begin
    debug_bus_cmd_ready = 1'b1;
    if(debug_bus_cmd_valid)begin
      case(_zz_182_)
        6'b000000 : begin
        end
        6'b000001 : begin
          if(debug_bus_cmd_payload_wr)begin
            debug_bus_cmd_ready = IBusCachedPlugin_injectionPort_ready;
          end
        end
        6'b010000 : begin
        end
        6'b010001 : begin
        end
        6'b010010 : begin
        end
        6'b010011 : begin
        end
        default : begin
        end
      endcase
    end
  end

  always @ (*) begin
    debug_bus_rsp_data = DebugPlugin_busReadDataReg;
    if((! _zz_141_))begin
      debug_bus_rsp_data[0] = DebugPlugin_resetIt;
      debug_bus_rsp_data[1] = DebugPlugin_haltIt;
      debug_bus_rsp_data[2] = DebugPlugin_isPipBusy;
      debug_bus_rsp_data[3] = DebugPlugin_haltedByBreak;
      debug_bus_rsp_data[4] = DebugPlugin_stepIt;
    end
  end

  always @ (*) begin
    IBusCachedPlugin_injectionPort_valid = 1'b0;
    if(debug_bus_cmd_valid)begin
      case(_zz_182_)
        6'b000000 : begin
        end
        6'b000001 : begin
          if(debug_bus_cmd_payload_wr)begin
            IBusCachedPlugin_injectionPort_valid = 1'b1;
          end
        end
        6'b010000 : begin
        end
        6'b010001 : begin
        end
        6'b010010 : begin
        end
        6'b010011 : begin
        end
        default : begin
        end
      endcase
    end
  end

  assign IBusCachedPlugin_injectionPort_payload = debug_bus_cmd_payload_data;
  assign _zz_20_ = ((! DebugPlugin_haltIt) && (decode_IS_EBREAK || ((((1'b0 || (DebugPlugin_hardwareBreakpoints_0_valid && (DebugPlugin_hardwareBreakpoints_0_pc == _zz_226_))) || (DebugPlugin_hardwareBreakpoints_1_valid && (DebugPlugin_hardwareBreakpoints_1_pc == _zz_227_))) || (DebugPlugin_hardwareBreakpoints_2_valid && (DebugPlugin_hardwareBreakpoints_2_pc == _zz_228_))) || (DebugPlugin_hardwareBreakpoints_3_valid && (DebugPlugin_hardwareBreakpoints_3_pc == _zz_229_)))));
  assign debug_resetOut = DebugPlugin_resetIt_regNext;
  assign _zz_39_ = _zz_58_;
  assign _zz_19_ = decode_BRANCH_CTRL;
  assign _zz_17_ = _zz_63_;
  assign _zz_27_ = decode_to_execute_BRANCH_CTRL;
  assign _zz_16_ = decode_ALU_CTRL;
  assign _zz_14_ = _zz_61_;
  assign _zz_42_ = decode_to_execute_ALU_CTRL;
  assign _zz_13_ = decode_SHIFT_CTRL;
  assign _zz_11_ = _zz_52_;
  assign _zz_30_ = decode_to_execute_SHIFT_CTRL;
  assign _zz_10_ = decode_ENV_CTRL;
  assign _zz_7_ = execute_ENV_CTRL;
  assign _zz_5_ = memory_ENV_CTRL;
  assign _zz_8_ = _zz_66_;
  assign _zz_22_ = decode_to_execute_ENV_CTRL;
  assign _zz_21_ = execute_to_memory_ENV_CTRL;
  assign _zz_25_ = memory_to_writeBack_ENV_CTRL;
  assign _zz_36_ = _zz_65_;
  assign _zz_3_ = decode_ALU_BITWISE_CTRL;
  assign _zz_1_ = _zz_57_;
  assign _zz_44_ = decode_to_execute_ALU_BITWISE_CTRL;
  assign decode_arbitration_isFlushed = (({writeBack_arbitration_flushNext,{memory_arbitration_flushNext,execute_arbitration_flushNext}} != (3'b000)) || ({writeBack_arbitration_flushIt,{memory_arbitration_flushIt,{execute_arbitration_flushIt,decode_arbitration_flushIt}}} != (4'b0000)));
  assign execute_arbitration_isFlushed = (({writeBack_arbitration_flushNext,memory_arbitration_flushNext} != (2'b00)) || ({writeBack_arbitration_flushIt,{memory_arbitration_flushIt,execute_arbitration_flushIt}} != (3'b000)));
  assign memory_arbitration_isFlushed = ((writeBack_arbitration_flushNext != (1'b0)) || ({writeBack_arbitration_flushIt,memory_arbitration_flushIt} != (2'b00)));
  assign writeBack_arbitration_isFlushed = (1'b0 || (writeBack_arbitration_flushIt != (1'b0)));
  assign decode_arbitration_isStuckByOthers = (decode_arbitration_haltByOther || (((1'b0 || execute_arbitration_isStuck) || memory_arbitration_isStuck) || writeBack_arbitration_isStuck));
  assign decode_arbitration_isStuck = (decode_arbitration_haltItself || decode_arbitration_isStuckByOthers);
  assign decode_arbitration_isMoving = ((! decode_arbitration_isStuck) && (! decode_arbitration_removeIt));
  assign decode_arbitration_isFiring = ((decode_arbitration_isValid && (! decode_arbitration_isStuck)) && (! decode_arbitration_removeIt));
  assign execute_arbitration_isStuckByOthers = (execute_arbitration_haltByOther || ((1'b0 || memory_arbitration_isStuck) || writeBack_arbitration_isStuck));
  assign execute_arbitration_isStuck = (execute_arbitration_haltItself || execute_arbitration_isStuckByOthers);
  assign execute_arbitration_isMoving = ((! execute_arbitration_isStuck) && (! execute_arbitration_removeIt));
  assign execute_arbitration_isFiring = ((execute_arbitration_isValid && (! execute_arbitration_isStuck)) && (! execute_arbitration_removeIt));
  assign memory_arbitration_isStuckByOthers = (memory_arbitration_haltByOther || (1'b0 || writeBack_arbitration_isStuck));
  assign memory_arbitration_isStuck = (memory_arbitration_haltItself || memory_arbitration_isStuckByOthers);
  assign memory_arbitration_isMoving = ((! memory_arbitration_isStuck) && (! memory_arbitration_removeIt));
  assign memory_arbitration_isFiring = ((memory_arbitration_isValid && (! memory_arbitration_isStuck)) && (! memory_arbitration_removeIt));
  assign writeBack_arbitration_isStuckByOthers = (writeBack_arbitration_haltByOther || 1'b0);
  assign writeBack_arbitration_isStuck = (writeBack_arbitration_haltItself || writeBack_arbitration_isStuckByOthers);
  assign writeBack_arbitration_isMoving = ((! writeBack_arbitration_isStuck) && (! writeBack_arbitration_removeIt));
  assign writeBack_arbitration_isFiring = ((writeBack_arbitration_isValid && (! writeBack_arbitration_isStuck)) && (! writeBack_arbitration_removeIt));
  always @ (*) begin
    IBusCachedPlugin_injectionPort_ready = 1'b0;
    case(_zz_142_)
      3'b000 : begin
      end
      3'b001 : begin
      end
      3'b010 : begin
      end
      3'b011 : begin
      end
      3'b100 : begin
        IBusCachedPlugin_injectionPort_ready = 1'b1;
      end
      default : begin
      end
    endcase
  end

  always @ (posedge clk_12M) begin
    if(clockCtrl_systemReset) begin
      IBusCachedPlugin_fetchPc_pcReg <= (32'b00100000000100000000000000000000);
      IBusCachedPlugin_fetchPc_booted <= 1'b0;
      IBusCachedPlugin_fetchPc_inc <= 1'b0;
      _zz_88_ <= 1'b0;
      _zz_89_ <= 1'b0;
      IBusCachedPlugin_injector_nextPcCalc_valids_0 <= 1'b0;
      IBusCachedPlugin_injector_nextPcCalc_valids_1 <= 1'b0;
      IBusCachedPlugin_injector_nextPcCalc_valids_2 <= 1'b0;
      IBusCachedPlugin_injector_nextPcCalc_valids_3 <= 1'b0;
      IBusCachedPlugin_injector_nextPcCalc_valids_4 <= 1'b0;
      IBusCachedPlugin_injector_decodeRemoved <= 1'b0;
      IBusCachedPlugin_rspCounter <= _zz_94_;
      IBusCachedPlugin_rspCounter <= (32'b00000000000000000000000000000000);
      execute_LightShifterPlugin_isActive <= 1'b0;
      _zz_136_ <= 1'b0;
      CsrPlugin_mstatus_MIE <= 1'b0;
      CsrPlugin_mstatus_MPIE <= 1'b0;
      CsrPlugin_mstatus_MPP <= (2'b11);
      CsrPlugin_mie_MEIE <= 1'b0;
      CsrPlugin_mie_MTIE <= 1'b0;
      CsrPlugin_mie_MSIE <= 1'b0;
      CsrPlugin_exceptionPortCtrl_exceptionValidsRegs_execute <= 1'b0;
      CsrPlugin_exceptionPortCtrl_exceptionValidsRegs_memory <= 1'b0;
      CsrPlugin_exceptionPortCtrl_exceptionValidsRegs_writeBack <= 1'b0;
      CsrPlugin_interrupt_valid <= 1'b0;
      CsrPlugin_hadException <= 1'b0;
      execute_CsrPlugin_wfiWake <= 1'b0;
      execute_arbitration_isValid <= 1'b0;
      memory_arbitration_isValid <= 1'b0;
      writeBack_arbitration_isValid <= 1'b0;
      _zz_142_ <= (3'b000);
    end else begin
      IBusCachedPlugin_fetchPc_booted <= 1'b1;
      if((IBusCachedPlugin_fetchPc_corrected || IBusCachedPlugin_fetchPc_pcRegPropagate))begin
        IBusCachedPlugin_fetchPc_inc <= 1'b0;
      end
      if((IBusCachedPlugin_fetchPc_output_valid && IBusCachedPlugin_fetchPc_output_ready))begin
        IBusCachedPlugin_fetchPc_inc <= 1'b1;
      end
      if(((! IBusCachedPlugin_fetchPc_output_valid) && IBusCachedPlugin_fetchPc_output_ready))begin
        IBusCachedPlugin_fetchPc_inc <= 1'b0;
      end
      if((IBusCachedPlugin_fetchPc_booted && ((IBusCachedPlugin_fetchPc_output_ready || IBusCachedPlugin_fetcherflushIt) || IBusCachedPlugin_fetchPc_pcRegPropagate)))begin
        IBusCachedPlugin_fetchPc_pcReg <= IBusCachedPlugin_fetchPc_pc;
      end
      if(IBusCachedPlugin_fetcherflushIt)begin
        _zz_88_ <= 1'b0;
      end
      if(_zz_86_)begin
        _zz_88_ <= IBusCachedPlugin_iBusRsp_stages_0_output_valid;
      end
      if(IBusCachedPlugin_iBusRsp_output_ready)begin
        _zz_89_ <= IBusCachedPlugin_iBusRsp_output_valid;
      end
      if(IBusCachedPlugin_fetcherflushIt)begin
        _zz_89_ <= 1'b0;
      end
      if(IBusCachedPlugin_fetcherflushIt)begin
        IBusCachedPlugin_injector_nextPcCalc_valids_0 <= 1'b0;
      end
      if((! (! IBusCachedPlugin_iBusRsp_stages_1_input_ready)))begin
        IBusCachedPlugin_injector_nextPcCalc_valids_0 <= 1'b1;
      end
      if(IBusCachedPlugin_fetcherflushIt)begin
        IBusCachedPlugin_injector_nextPcCalc_valids_1 <= 1'b0;
      end
      if((! (! IBusCachedPlugin_injector_decodeInput_ready)))begin
        IBusCachedPlugin_injector_nextPcCalc_valids_1 <= IBusCachedPlugin_injector_nextPcCalc_valids_0;
      end
      if(IBusCachedPlugin_fetcherflushIt)begin
        IBusCachedPlugin_injector_nextPcCalc_valids_1 <= 1'b0;
      end
      if(IBusCachedPlugin_fetcherflushIt)begin
        IBusCachedPlugin_injector_nextPcCalc_valids_2 <= 1'b0;
      end
      if((! execute_arbitration_isStuck))begin
        IBusCachedPlugin_injector_nextPcCalc_valids_2 <= IBusCachedPlugin_injector_nextPcCalc_valids_1;
      end
      if(IBusCachedPlugin_fetcherflushIt)begin
        IBusCachedPlugin_injector_nextPcCalc_valids_2 <= 1'b0;
      end
      if(IBusCachedPlugin_fetcherflushIt)begin
        IBusCachedPlugin_injector_nextPcCalc_valids_3 <= 1'b0;
      end
      if((! memory_arbitration_isStuck))begin
        IBusCachedPlugin_injector_nextPcCalc_valids_3 <= IBusCachedPlugin_injector_nextPcCalc_valids_2;
      end
      if(IBusCachedPlugin_fetcherflushIt)begin
        IBusCachedPlugin_injector_nextPcCalc_valids_3 <= 1'b0;
      end
      if(IBusCachedPlugin_fetcherflushIt)begin
        IBusCachedPlugin_injector_nextPcCalc_valids_4 <= 1'b0;
      end
      if((! writeBack_arbitration_isStuck))begin
        IBusCachedPlugin_injector_nextPcCalc_valids_4 <= IBusCachedPlugin_injector_nextPcCalc_valids_3;
      end
      if(IBusCachedPlugin_fetcherflushIt)begin
        IBusCachedPlugin_injector_nextPcCalc_valids_4 <= 1'b0;
      end
      if(decode_arbitration_removeIt)begin
        IBusCachedPlugin_injector_decodeRemoved <= 1'b1;
      end
      if(IBusCachedPlugin_fetcherflushIt)begin
        IBusCachedPlugin_injector_decodeRemoved <= 1'b0;
      end
      if(iBus_rsp_valid)begin
        IBusCachedPlugin_rspCounter <= (IBusCachedPlugin_rspCounter + (32'b00000000000000000000000000000001));
      end
      if(_zz_163_)begin
        if(_zz_167_)begin
          execute_LightShifterPlugin_isActive <= 1'b1;
          if(execute_LightShifterPlugin_done)begin
            execute_LightShifterPlugin_isActive <= 1'b0;
          end
        end
      end
      if(execute_arbitration_removeIt)begin
        execute_LightShifterPlugin_isActive <= 1'b0;
      end
      _zz_136_ <= _zz_135_;
      if((! execute_arbitration_isStuck))begin
        CsrPlugin_exceptionPortCtrl_exceptionValidsRegs_execute <= 1'b0;
      end else begin
        CsrPlugin_exceptionPortCtrl_exceptionValidsRegs_execute <= CsrPlugin_exceptionPortCtrl_exceptionValids_execute;
      end
      if((! memory_arbitration_isStuck))begin
        CsrPlugin_exceptionPortCtrl_exceptionValidsRegs_memory <= (CsrPlugin_exceptionPortCtrl_exceptionValids_execute && (! execute_arbitration_isStuck));
      end else begin
        CsrPlugin_exceptionPortCtrl_exceptionValidsRegs_memory <= CsrPlugin_exceptionPortCtrl_exceptionValids_memory;
      end
      if((! writeBack_arbitration_isStuck))begin
        CsrPlugin_exceptionPortCtrl_exceptionValidsRegs_writeBack <= (CsrPlugin_exceptionPortCtrl_exceptionValids_memory && (! memory_arbitration_isStuck));
      end else begin
        CsrPlugin_exceptionPortCtrl_exceptionValidsRegs_writeBack <= 1'b0;
      end
      CsrPlugin_interrupt_valid <= 1'b0;
      if(_zz_183_)begin
        if(_zz_184_)begin
          CsrPlugin_interrupt_valid <= 1'b1;
        end
        if(_zz_185_)begin
          CsrPlugin_interrupt_valid <= 1'b1;
        end
        if(_zz_186_)begin
          CsrPlugin_interrupt_valid <= 1'b1;
        end
      end
      CsrPlugin_hadException <= CsrPlugin_exception;
      if(_zz_170_)begin
        case(CsrPlugin_targetPrivilege)
          2'b11 : begin
            CsrPlugin_mstatus_MIE <= 1'b0;
            CsrPlugin_mstatus_MPIE <= CsrPlugin_mstatus_MIE;
            CsrPlugin_mstatus_MPP <= CsrPlugin_privilege;
          end
          default : begin
          end
        endcase
      end
      if(_zz_171_)begin
        case(_zz_173_)
          2'b11 : begin
            CsrPlugin_mstatus_MPP <= (2'b00);
            CsrPlugin_mstatus_MIE <= CsrPlugin_mstatus_MPIE;
            CsrPlugin_mstatus_MPIE <= 1'b1;
          end
          default : begin
          end
        endcase
      end
      execute_CsrPlugin_wfiWake <= ({_zz_140_,{_zz_139_,_zz_138_}} != (3'b000));
      if(((! execute_arbitration_isStuck) || execute_arbitration_removeIt))begin
        execute_arbitration_isValid <= 1'b0;
      end
      if(((! decode_arbitration_isStuck) && (! decode_arbitration_removeIt)))begin
        execute_arbitration_isValid <= decode_arbitration_isValid;
      end
      if(((! memory_arbitration_isStuck) || memory_arbitration_removeIt))begin
        memory_arbitration_isValid <= 1'b0;
      end
      if(((! execute_arbitration_isStuck) && (! execute_arbitration_removeIt)))begin
        memory_arbitration_isValid <= execute_arbitration_isValid;
      end
      if(((! writeBack_arbitration_isStuck) || writeBack_arbitration_removeIt))begin
        writeBack_arbitration_isValid <= 1'b0;
      end
      if(((! memory_arbitration_isStuck) && (! memory_arbitration_removeIt)))begin
        writeBack_arbitration_isValid <= memory_arbitration_isValid;
      end
      case(_zz_142_)
        3'b000 : begin
          if(IBusCachedPlugin_injectionPort_valid)begin
            _zz_142_ <= (3'b001);
          end
        end
        3'b001 : begin
          _zz_142_ <= (3'b010);
        end
        3'b010 : begin
          _zz_142_ <= (3'b011);
        end
        3'b011 : begin
          if((! decode_arbitration_isStuck))begin
            _zz_142_ <= (3'b100);
          end
        end
        3'b100 : begin
          _zz_142_ <= (3'b000);
        end
        default : begin
        end
      endcase
      case(execute_CsrPlugin_csrAddress)
        12'b001100000000 : begin
          if(execute_CsrPlugin_writeEnable)begin
            CsrPlugin_mstatus_MPP <= execute_CsrPlugin_writeData[12 : 11];
            CsrPlugin_mstatus_MPIE <= _zz_230_[0];
            CsrPlugin_mstatus_MIE <= _zz_231_[0];
          end
        end
        12'b001101000001 : begin
        end
        12'b001100000101 : begin
        end
        12'b001101000100 : begin
        end
        12'b001100000100 : begin
          if(execute_CsrPlugin_writeEnable)begin
            CsrPlugin_mie_MEIE <= _zz_233_[0];
            CsrPlugin_mie_MTIE <= _zz_234_[0];
            CsrPlugin_mie_MSIE <= _zz_235_[0];
          end
        end
        12'b001101000010 : begin
        end
        default : begin
        end
      endcase
    end
  end

  always @ (posedge clk_12M) begin
    if(IBusCachedPlugin_iBusRsp_output_ready)begin
      _zz_90_ <= IBusCachedPlugin_iBusRsp_output_payload_pc;
      _zz_91_ <= IBusCachedPlugin_iBusRsp_output_payload_rsp_error;
      _zz_92_ <= IBusCachedPlugin_iBusRsp_output_payload_rsp_inst;
      _zz_93_ <= IBusCachedPlugin_iBusRsp_output_payload_isRvc;
    end
    if(IBusCachedPlugin_injector_decodeInput_ready)begin
      IBusCachedPlugin_injector_formal_rawInDecode <= IBusCachedPlugin_iBusRsp_output_payload_rsp_inst;
    end
    if(IBusCachedPlugin_iBusRsp_stages_1_input_ready)begin
      IBusCachedPlugin_s1_tightlyCoupledHit <= IBusCachedPlugin_s0_tightlyCoupledHit;
    end
    if(!(! (((dBus_rsp_ready && memory_MEMORY_ENABLE) && memory_arbitration_isValid) && memory_arbitration_isStuck))) begin
      $display("ERROR DBusSimplePlugin doesn't allow memory stage stall when read happend");
    end
    if(!(! (((writeBack_arbitration_isValid && writeBack_MEMORY_ENABLE) && (! writeBack_MEMORY_STORE)) && writeBack_arbitration_isStuck))) begin
      $display("ERROR DBusSimplePlugin doesn't allow writeback stage stall when read happend");
    end
    if(_zz_163_)begin
      if(_zz_167_)begin
        execute_LightShifterPlugin_amplitudeReg <= (execute_LightShifterPlugin_amplitude - (5'b00001));
      end
    end
    if(_zz_135_)begin
      _zz_137_ <= _zz_45_[11 : 7];
    end
    CsrPlugin_mip_MEIP <= externalInterrupt;
    CsrPlugin_mip_MTIP <= timerInterrupt;
    CsrPlugin_mip_MSIP <= softwareInterrupt;
    CsrPlugin_mcycle <= (CsrPlugin_mcycle + (64'b0000000000000000000000000000000000000000000000000000000000000001));
    if(writeBack_arbitration_isFiring)begin
      CsrPlugin_minstret <= (CsrPlugin_minstret + (64'b0000000000000000000000000000000000000000000000000000000000000001));
    end
    if(CsrPlugin_selfException_valid)begin
      CsrPlugin_exceptionPortCtrl_exceptionContext_code <= CsrPlugin_selfException_payload_code;
      CsrPlugin_exceptionPortCtrl_exceptionContext_badAddr <= CsrPlugin_selfException_payload_badAddr;
    end
    if(_zz_183_)begin
      if(_zz_184_)begin
        CsrPlugin_interrupt_code <= (4'b0111);
        CsrPlugin_interrupt_targetPrivilege <= (2'b11);
      end
      if(_zz_185_)begin
        CsrPlugin_interrupt_code <= (4'b0011);
        CsrPlugin_interrupt_targetPrivilege <= (2'b11);
      end
      if(_zz_186_)begin
        CsrPlugin_interrupt_code <= (4'b1011);
        CsrPlugin_interrupt_targetPrivilege <= (2'b11);
      end
    end
    if(_zz_170_)begin
      case(CsrPlugin_targetPrivilege)
        2'b11 : begin
          CsrPlugin_mcause_interrupt <= (! CsrPlugin_hadException);
          CsrPlugin_mcause_exceptionCode <= CsrPlugin_trapCause;
          CsrPlugin_mepc <= writeBack_PC;
          if(CsrPlugin_hadException)begin
            CsrPlugin_mtval <= CsrPlugin_exceptionPortCtrl_exceptionContext_badAddr;
          end
        end
        default : begin
        end
      endcase
    end
    if((! memory_arbitration_isStuck))begin
      execute_to_memory_BRANCH_CALC <= execute_BRANCH_CALC;
    end
    if((! execute_arbitration_isStuck))begin
      decode_to_execute_MEMORY_ENABLE <= decode_MEMORY_ENABLE;
    end
    if((! memory_arbitration_isStuck))begin
      execute_to_memory_MEMORY_ENABLE <= execute_MEMORY_ENABLE;
    end
    if((! writeBack_arbitration_isStuck))begin
      memory_to_writeBack_MEMORY_ENABLE <= memory_MEMORY_ENABLE;
    end
    if((! execute_arbitration_isStuck))begin
      decode_to_execute_CSR_READ_OPCODE <= decode_CSR_READ_OPCODE;
    end
    if((! execute_arbitration_isStuck))begin
      decode_to_execute_SRC_LESS_UNSIGNED <= decode_SRC_LESS_UNSIGNED;
    end
    if((! execute_arbitration_isStuck))begin
      decode_to_execute_SRC_USE_SUB_LESS <= decode_SRC_USE_SUB_LESS;
    end
    if((! execute_arbitration_isStuck))begin
      decode_to_execute_SRC2_FORCE_ZERO <= decode_SRC2_FORCE_ZERO;
    end
    if((! execute_arbitration_isStuck))begin
      decode_to_execute_IS_CSR <= decode_IS_CSR;
    end
    if((! execute_arbitration_isStuck))begin
      decode_to_execute_SRC1 <= decode_SRC1;
    end
    if((! memory_arbitration_isStuck))begin
      execute_to_memory_BRANCH_DO <= execute_BRANCH_DO;
    end
    if((! execute_arbitration_isStuck))begin
      decode_to_execute_RS2 <= _zz_35_;
    end
    if((! execute_arbitration_isStuck))begin
      decode_to_execute_DO_EBREAK <= decode_DO_EBREAK;
    end
    if((! execute_arbitration_isStuck))begin
      decode_to_execute_RS1 <= _zz_38_;
    end
    if((! writeBack_arbitration_isStuck))begin
      memory_to_writeBack_MEMORY_READ_DATA <= memory_MEMORY_READ_DATA;
    end
    if(((! memory_arbitration_isStuck) && (! execute_arbitration_isStuckByOthers)))begin
      execute_to_memory_REGFILE_WRITE_DATA <= _zz_29_;
    end
    if((! writeBack_arbitration_isStuck))begin
      memory_to_writeBack_REGFILE_WRITE_DATA <= memory_REGFILE_WRITE_DATA;
    end
    if((! execute_arbitration_isStuck))begin
      decode_to_execute_REGFILE_WRITE_VALID <= decode_REGFILE_WRITE_VALID;
    end
    if((! memory_arbitration_isStuck))begin
      execute_to_memory_REGFILE_WRITE_VALID <= execute_REGFILE_WRITE_VALID;
    end
    if((! writeBack_arbitration_isStuck))begin
      memory_to_writeBack_REGFILE_WRITE_VALID <= memory_REGFILE_WRITE_VALID;
    end
    if((! execute_arbitration_isStuck))begin
      decode_to_execute_FORMAL_PC_NEXT <= _zz_75_;
    end
    if((! memory_arbitration_isStuck))begin
      execute_to_memory_FORMAL_PC_NEXT <= execute_FORMAL_PC_NEXT;
    end
    if((! writeBack_arbitration_isStuck))begin
      memory_to_writeBack_FORMAL_PC_NEXT <= _zz_74_;
    end
    if((! execute_arbitration_isStuck))begin
      decode_to_execute_BRANCH_CTRL <= _zz_18_;
    end
    if((! execute_arbitration_isStuck))begin
      decode_to_execute_INSTRUCTION <= decode_INSTRUCTION;
    end
    if((! memory_arbitration_isStuck))begin
      execute_to_memory_INSTRUCTION <= execute_INSTRUCTION;
    end
    if((! writeBack_arbitration_isStuck))begin
      memory_to_writeBack_INSTRUCTION <= memory_INSTRUCTION;
    end
    if((! execute_arbitration_isStuck))begin
      decode_to_execute_ALU_CTRL <= _zz_15_;
    end
    if((! execute_arbitration_isStuck))begin
      decode_to_execute_SHIFT_CTRL <= _zz_12_;
    end
    if((! execute_arbitration_isStuck))begin
      decode_to_execute_ENV_CTRL <= _zz_9_;
    end
    if((! memory_arbitration_isStuck))begin
      execute_to_memory_ENV_CTRL <= _zz_6_;
    end
    if((! writeBack_arbitration_isStuck))begin
      memory_to_writeBack_ENV_CTRL <= _zz_4_;
    end
    if((! execute_arbitration_isStuck))begin
      decode_to_execute_CSR_WRITE_OPCODE <= decode_CSR_WRITE_OPCODE;
    end
    if((! execute_arbitration_isStuck))begin
      decode_to_execute_PC <= _zz_34_;
    end
    if((! memory_arbitration_isStuck))begin
      execute_to_memory_PC <= execute_PC;
    end
    if(((! writeBack_arbitration_isStuck) && (! CsrPlugin_exceptionPortCtrl_exceptionValids_writeBack)))begin
      memory_to_writeBack_PC <= memory_PC;
    end
    if((! execute_arbitration_isStuck))begin
      decode_to_execute_BYPASSABLE_MEMORY_STAGE <= decode_BYPASSABLE_MEMORY_STAGE;
    end
    if((! memory_arbitration_isStuck))begin
      execute_to_memory_BYPASSABLE_MEMORY_STAGE <= execute_BYPASSABLE_MEMORY_STAGE;
    end
    if((! execute_arbitration_isStuck))begin
      decode_to_execute_SRC2 <= decode_SRC2;
    end
    if((! execute_arbitration_isStuck))begin
      decode_to_execute_BYPASSABLE_EXECUTE_STAGE <= decode_BYPASSABLE_EXECUTE_STAGE;
    end
    if((! execute_arbitration_isStuck))begin
      decode_to_execute_MEMORY_STORE <= decode_MEMORY_STORE;
    end
    if((! memory_arbitration_isStuck))begin
      execute_to_memory_MEMORY_STORE <= execute_MEMORY_STORE;
    end
    if((! writeBack_arbitration_isStuck))begin
      memory_to_writeBack_MEMORY_STORE <= memory_MEMORY_STORE;
    end
    if((! memory_arbitration_isStuck))begin
      execute_to_memory_MEMORY_ADDRESS_LOW <= execute_MEMORY_ADDRESS_LOW;
    end
    if((! writeBack_arbitration_isStuck))begin
      memory_to_writeBack_MEMORY_ADDRESS_LOW <= memory_MEMORY_ADDRESS_LOW;
    end
    if((! execute_arbitration_isStuck))begin
      decode_to_execute_ALU_BITWISE_CTRL <= _zz_2_;
    end
    if((_zz_142_ != (3'b000)))begin
      _zz_92_ <= IBusCachedPlugin_injectionPort_payload;
    end
    case(execute_CsrPlugin_csrAddress)
      12'b001100000000 : begin
      end
      12'b001101000001 : begin
        if(execute_CsrPlugin_writeEnable)begin
          CsrPlugin_mepc <= execute_CsrPlugin_writeData[31 : 0];
        end
      end
      12'b001100000101 : begin
        if(execute_CsrPlugin_writeEnable)begin
          CsrPlugin_mtvec_base <= execute_CsrPlugin_writeData[31 : 2];
          CsrPlugin_mtvec_mode <= execute_CsrPlugin_writeData[1 : 0];
        end
      end
      12'b001101000100 : begin
        if(execute_CsrPlugin_writeEnable)begin
          CsrPlugin_mip_MSIP <= _zz_232_[0];
        end
      end
      12'b001100000100 : begin
      end
      12'b001101000010 : begin
      end
      default : begin
      end
    endcase
  end

  always @ (posedge clk_12M) begin
    DebugPlugin_firstCycle <= 1'b0;
    if(debug_bus_cmd_ready)begin
      DebugPlugin_firstCycle <= 1'b1;
    end
    DebugPlugin_secondCycle <= DebugPlugin_firstCycle;
    DebugPlugin_isPipBusy <= (({writeBack_arbitration_isValid,{memory_arbitration_isValid,{execute_arbitration_isValid,decode_arbitration_isValid}}} != (4'b0000)) || IBusCachedPlugin_incomingInstruction);
    if(writeBack_arbitration_isValid)begin
      DebugPlugin_busReadDataReg <= _zz_70_;
    end
    _zz_141_ <= debug_bus_cmd_payload_address[2];
    if(debug_bus_cmd_valid)begin
      case(_zz_182_)
        6'b000000 : begin
        end
        6'b000001 : begin
        end
        6'b010000 : begin
          if(debug_bus_cmd_payload_wr)begin
            DebugPlugin_hardwareBreakpoints_0_pc <= debug_bus_cmd_payload_data[31 : 1];
          end
        end
        6'b010001 : begin
          if(debug_bus_cmd_payload_wr)begin
            DebugPlugin_hardwareBreakpoints_1_pc <= debug_bus_cmd_payload_data[31 : 1];
          end
        end
        6'b010010 : begin
          if(debug_bus_cmd_payload_wr)begin
            DebugPlugin_hardwareBreakpoints_2_pc <= debug_bus_cmd_payload_data[31 : 1];
          end
        end
        6'b010011 : begin
          if(debug_bus_cmd_payload_wr)begin
            DebugPlugin_hardwareBreakpoints_3_pc <= debug_bus_cmd_payload_data[31 : 1];
          end
        end
        default : begin
        end
      endcase
    end
    if(_zz_168_)begin
      DebugPlugin_busReadDataReg <= execute_PC;
    end
    DebugPlugin_resetIt_regNext <= DebugPlugin_resetIt;
  end

  always @ (posedge clk_12M) begin
    if(clockCtrl_resetUnbuffered_regNext) begin
      DebugPlugin_resetIt <= 1'b0;
      DebugPlugin_haltIt <= 1'b0;
      DebugPlugin_stepIt <= 1'b0;
      DebugPlugin_godmode <= 1'b0;
      DebugPlugin_haltedByBreak <= 1'b0;
      DebugPlugin_hardwareBreakpoints_0_valid <= 1'b0;
      DebugPlugin_hardwareBreakpoints_1_valid <= 1'b0;
      DebugPlugin_hardwareBreakpoints_2_valid <= 1'b0;
      DebugPlugin_hardwareBreakpoints_3_valid <= 1'b0;
    end else begin
      if((DebugPlugin_haltIt && (! DebugPlugin_isPipBusy)))begin
        DebugPlugin_godmode <= 1'b1;
      end
      if(debug_bus_cmd_valid)begin
        case(_zz_182_)
          6'b000000 : begin
            if(debug_bus_cmd_payload_wr)begin
              DebugPlugin_stepIt <= debug_bus_cmd_payload_data[4];
              if(debug_bus_cmd_payload_data[16])begin
                DebugPlugin_resetIt <= 1'b1;
              end
              if(debug_bus_cmd_payload_data[24])begin
                DebugPlugin_resetIt <= 1'b0;
              end
              if(debug_bus_cmd_payload_data[17])begin
                DebugPlugin_haltIt <= 1'b1;
              end
              if(debug_bus_cmd_payload_data[25])begin
                DebugPlugin_haltIt <= 1'b0;
              end
              if(debug_bus_cmd_payload_data[25])begin
                DebugPlugin_haltedByBreak <= 1'b0;
              end
              if(debug_bus_cmd_payload_data[25])begin
                DebugPlugin_godmode <= 1'b0;
              end
            end
          end
          6'b000001 : begin
          end
          6'b010000 : begin
            if(debug_bus_cmd_payload_wr)begin
              DebugPlugin_hardwareBreakpoints_0_valid <= _zz_222_[0];
            end
          end
          6'b010001 : begin
            if(debug_bus_cmd_payload_wr)begin
              DebugPlugin_hardwareBreakpoints_1_valid <= _zz_223_[0];
            end
          end
          6'b010010 : begin
            if(debug_bus_cmd_payload_wr)begin
              DebugPlugin_hardwareBreakpoints_2_valid <= _zz_224_[0];
            end
          end
          6'b010011 : begin
            if(debug_bus_cmd_payload_wr)begin
              DebugPlugin_hardwareBreakpoints_3_valid <= _zz_225_[0];
            end
          end
          default : begin
          end
        endcase
      end
      if(_zz_168_)begin
        if(_zz_169_)begin
          DebugPlugin_haltIt <= 1'b1;
          DebugPlugin_haltedByBreak <= 1'b1;
        end
      end
      if(_zz_172_)begin
        if(decode_arbitration_isValid)begin
          DebugPlugin_haltIt <= 1'b1;
        end
      end
    end
  end

endmodule

module JtagBridge (
      input   io_jtag_tms,
      input   io_jtag_tdi,
      output  io_jtag_tdo,
      input   io_jtag_tck,
      output  io_remote_cmd_valid,
      input   io_remote_cmd_ready,
      output  io_remote_cmd_payload_last,
      output [0:0] io_remote_cmd_payload_fragment,
      input   io_remote_rsp_valid,
      output  io_remote_rsp_ready,
      input   io_remote_rsp_payload_error,
      input  [31:0] io_remote_rsp_payload_data,
      input   clk_12M,
      input   clockCtrl_resetUnbuffered_regNext);
  wire  flowCCByToggle_1__io_output_valid;
  wire  flowCCByToggle_1__io_output_payload_last;
  wire [0:0] flowCCByToggle_1__io_output_payload_fragment;
  wire  _zz_2_;
  wire  _zz_3_;
  wire [3:0] _zz_4_;
  wire [3:0] _zz_5_;
  wire [3:0] _zz_6_;
  wire  system_cmd_valid;
  wire  system_cmd_payload_last;
  wire [0:0] system_cmd_payload_fragment;
  reg  system_rsp_valid;
  reg  system_rsp_payload_error;
  reg [31:0] system_rsp_payload_data;
  wire `JtagState_defaultEncoding_type jtag_tap_fsm_stateNext;
  reg `JtagState_defaultEncoding_type jtag_tap_fsm_state = `JtagState_defaultEncoding_RESET;
  reg `JtagState_defaultEncoding_type _zz_1_;
  reg [3:0] jtag_tap_instruction;
  reg [3:0] jtag_tap_instructionShift;
  reg  jtag_tap_bypass;
  reg  jtag_tap_tdoUnbufferd;
  reg  jtag_tap_tdoUnbufferd_regNext;
  wire [0:0] jtag_idcodeArea_instructionId;
  wire  jtag_idcodeArea_instructionHit;
  reg [31:0] jtag_idcodeArea_shifter;
  wire [1:0] jtag_writeArea_instructionId;
  wire  jtag_writeArea_instructionHit;
  reg  jtag_writeArea_source_valid;
  wire  jtag_writeArea_source_payload_last;
  wire [0:0] jtag_writeArea_source_payload_fragment;
  wire [1:0] jtag_readArea_instructionId;
  wire  jtag_readArea_instructionHit;
  reg [33:0] jtag_readArea_shifter;
  `ifndef SYNTHESIS
  reg [79:0] jtag_tap_fsm_stateNext_string;
  reg [79:0] jtag_tap_fsm_state_string;
  reg [79:0] _zz_1__string;
  `endif

  assign _zz_2_ = (jtag_tap_fsm_state == `JtagState_defaultEncoding_DR_SHIFT);
  assign _zz_3_ = (jtag_tap_fsm_state == `JtagState_defaultEncoding_DR_SHIFT);
  assign _zz_4_ = {3'd0, jtag_idcodeArea_instructionId};
  assign _zz_5_ = {2'd0, jtag_writeArea_instructionId};
  assign _zz_6_ = {2'd0, jtag_readArea_instructionId};
  FlowCCByToggle flowCCByToggle_1_ ( 
    .io_input_valid(jtag_writeArea_source_valid),
    .io_input_payload_last(jtag_writeArea_source_payload_last),
    .io_input_payload_fragment(jtag_writeArea_source_payload_fragment),
    .io_output_valid(flowCCByToggle_1__io_output_valid),
    .io_output_payload_last(flowCCByToggle_1__io_output_payload_last),
    .io_output_payload_fragment(flowCCByToggle_1__io_output_payload_fragment),
    .io_jtag_tck(io_jtag_tck),
    .clk_12M(clk_12M),
    .clockCtrl_resetUnbuffered_regNext(clockCtrl_resetUnbuffered_regNext) 
  );
  `ifndef SYNTHESIS
  always @(*) begin
    case(jtag_tap_fsm_stateNext)
      `JtagState_defaultEncoding_RESET : jtag_tap_fsm_stateNext_string = "RESET     ";
      `JtagState_defaultEncoding_IDLE : jtag_tap_fsm_stateNext_string = "IDLE      ";
      `JtagState_defaultEncoding_IR_SELECT : jtag_tap_fsm_stateNext_string = "IR_SELECT ";
      `JtagState_defaultEncoding_IR_CAPTURE : jtag_tap_fsm_stateNext_string = "IR_CAPTURE";
      `JtagState_defaultEncoding_IR_SHIFT : jtag_tap_fsm_stateNext_string = "IR_SHIFT  ";
      `JtagState_defaultEncoding_IR_EXIT1 : jtag_tap_fsm_stateNext_string = "IR_EXIT1  ";
      `JtagState_defaultEncoding_IR_PAUSE : jtag_tap_fsm_stateNext_string = "IR_PAUSE  ";
      `JtagState_defaultEncoding_IR_EXIT2 : jtag_tap_fsm_stateNext_string = "IR_EXIT2  ";
      `JtagState_defaultEncoding_IR_UPDATE : jtag_tap_fsm_stateNext_string = "IR_UPDATE ";
      `JtagState_defaultEncoding_DR_SELECT : jtag_tap_fsm_stateNext_string = "DR_SELECT ";
      `JtagState_defaultEncoding_DR_CAPTURE : jtag_tap_fsm_stateNext_string = "DR_CAPTURE";
      `JtagState_defaultEncoding_DR_SHIFT : jtag_tap_fsm_stateNext_string = "DR_SHIFT  ";
      `JtagState_defaultEncoding_DR_EXIT1 : jtag_tap_fsm_stateNext_string = "DR_EXIT1  ";
      `JtagState_defaultEncoding_DR_PAUSE : jtag_tap_fsm_stateNext_string = "DR_PAUSE  ";
      `JtagState_defaultEncoding_DR_EXIT2 : jtag_tap_fsm_stateNext_string = "DR_EXIT2  ";
      `JtagState_defaultEncoding_DR_UPDATE : jtag_tap_fsm_stateNext_string = "DR_UPDATE ";
      default : jtag_tap_fsm_stateNext_string = "??????????";
    endcase
  end
  always @(*) begin
    case(jtag_tap_fsm_state)
      `JtagState_defaultEncoding_RESET : jtag_tap_fsm_state_string = "RESET     ";
      `JtagState_defaultEncoding_IDLE : jtag_tap_fsm_state_string = "IDLE      ";
      `JtagState_defaultEncoding_IR_SELECT : jtag_tap_fsm_state_string = "IR_SELECT ";
      `JtagState_defaultEncoding_IR_CAPTURE : jtag_tap_fsm_state_string = "IR_CAPTURE";
      `JtagState_defaultEncoding_IR_SHIFT : jtag_tap_fsm_state_string = "IR_SHIFT  ";
      `JtagState_defaultEncoding_IR_EXIT1 : jtag_tap_fsm_state_string = "IR_EXIT1  ";
      `JtagState_defaultEncoding_IR_PAUSE : jtag_tap_fsm_state_string = "IR_PAUSE  ";
      `JtagState_defaultEncoding_IR_EXIT2 : jtag_tap_fsm_state_string = "IR_EXIT2  ";
      `JtagState_defaultEncoding_IR_UPDATE : jtag_tap_fsm_state_string = "IR_UPDATE ";
      `JtagState_defaultEncoding_DR_SELECT : jtag_tap_fsm_state_string = "DR_SELECT ";
      `JtagState_defaultEncoding_DR_CAPTURE : jtag_tap_fsm_state_string = "DR_CAPTURE";
      `JtagState_defaultEncoding_DR_SHIFT : jtag_tap_fsm_state_string = "DR_SHIFT  ";
      `JtagState_defaultEncoding_DR_EXIT1 : jtag_tap_fsm_state_string = "DR_EXIT1  ";
      `JtagState_defaultEncoding_DR_PAUSE : jtag_tap_fsm_state_string = "DR_PAUSE  ";
      `JtagState_defaultEncoding_DR_EXIT2 : jtag_tap_fsm_state_string = "DR_EXIT2  ";
      `JtagState_defaultEncoding_DR_UPDATE : jtag_tap_fsm_state_string = "DR_UPDATE ";
      default : jtag_tap_fsm_state_string = "??????????";
    endcase
  end
  always @(*) begin
    case(_zz_1_)
      `JtagState_defaultEncoding_RESET : _zz_1__string = "RESET     ";
      `JtagState_defaultEncoding_IDLE : _zz_1__string = "IDLE      ";
      `JtagState_defaultEncoding_IR_SELECT : _zz_1__string = "IR_SELECT ";
      `JtagState_defaultEncoding_IR_CAPTURE : _zz_1__string = "IR_CAPTURE";
      `JtagState_defaultEncoding_IR_SHIFT : _zz_1__string = "IR_SHIFT  ";
      `JtagState_defaultEncoding_IR_EXIT1 : _zz_1__string = "IR_EXIT1  ";
      `JtagState_defaultEncoding_IR_PAUSE : _zz_1__string = "IR_PAUSE  ";
      `JtagState_defaultEncoding_IR_EXIT2 : _zz_1__string = "IR_EXIT2  ";
      `JtagState_defaultEncoding_IR_UPDATE : _zz_1__string = "IR_UPDATE ";
      `JtagState_defaultEncoding_DR_SELECT : _zz_1__string = "DR_SELECT ";
      `JtagState_defaultEncoding_DR_CAPTURE : _zz_1__string = "DR_CAPTURE";
      `JtagState_defaultEncoding_DR_SHIFT : _zz_1__string = "DR_SHIFT  ";
      `JtagState_defaultEncoding_DR_EXIT1 : _zz_1__string = "DR_EXIT1  ";
      `JtagState_defaultEncoding_DR_PAUSE : _zz_1__string = "DR_PAUSE  ";
      `JtagState_defaultEncoding_DR_EXIT2 : _zz_1__string = "DR_EXIT2  ";
      `JtagState_defaultEncoding_DR_UPDATE : _zz_1__string = "DR_UPDATE ";
      default : _zz_1__string = "??????????";
    endcase
  end
  `endif

  assign io_remote_cmd_valid = system_cmd_valid;
  assign io_remote_cmd_payload_last = system_cmd_payload_last;
  assign io_remote_cmd_payload_fragment = system_cmd_payload_fragment;
  assign io_remote_rsp_ready = 1'b1;
  always @ (*) begin
    case(jtag_tap_fsm_state)
      `JtagState_defaultEncoding_IDLE : begin
        _zz_1_ = (io_jtag_tms ? `JtagState_defaultEncoding_DR_SELECT : `JtagState_defaultEncoding_IDLE);
      end
      `JtagState_defaultEncoding_IR_SELECT : begin
        _zz_1_ = (io_jtag_tms ? `JtagState_defaultEncoding_RESET : `JtagState_defaultEncoding_IR_CAPTURE);
      end
      `JtagState_defaultEncoding_IR_CAPTURE : begin
        _zz_1_ = (io_jtag_tms ? `JtagState_defaultEncoding_IR_EXIT1 : `JtagState_defaultEncoding_IR_SHIFT);
      end
      `JtagState_defaultEncoding_IR_SHIFT : begin
        _zz_1_ = (io_jtag_tms ? `JtagState_defaultEncoding_IR_EXIT1 : `JtagState_defaultEncoding_IR_SHIFT);
      end
      `JtagState_defaultEncoding_IR_EXIT1 : begin
        _zz_1_ = (io_jtag_tms ? `JtagState_defaultEncoding_IR_UPDATE : `JtagState_defaultEncoding_IR_PAUSE);
      end
      `JtagState_defaultEncoding_IR_PAUSE : begin
        _zz_1_ = (io_jtag_tms ? `JtagState_defaultEncoding_IR_EXIT2 : `JtagState_defaultEncoding_IR_PAUSE);
      end
      `JtagState_defaultEncoding_IR_EXIT2 : begin
        _zz_1_ = (io_jtag_tms ? `JtagState_defaultEncoding_IR_UPDATE : `JtagState_defaultEncoding_IR_SHIFT);
      end
      `JtagState_defaultEncoding_IR_UPDATE : begin
        _zz_1_ = (io_jtag_tms ? `JtagState_defaultEncoding_DR_SELECT : `JtagState_defaultEncoding_IDLE);
      end
      `JtagState_defaultEncoding_DR_SELECT : begin
        _zz_1_ = (io_jtag_tms ? `JtagState_defaultEncoding_IR_SELECT : `JtagState_defaultEncoding_DR_CAPTURE);
      end
      `JtagState_defaultEncoding_DR_CAPTURE : begin
        _zz_1_ = (io_jtag_tms ? `JtagState_defaultEncoding_DR_EXIT1 : `JtagState_defaultEncoding_DR_SHIFT);
      end
      `JtagState_defaultEncoding_DR_SHIFT : begin
        _zz_1_ = (io_jtag_tms ? `JtagState_defaultEncoding_DR_EXIT1 : `JtagState_defaultEncoding_DR_SHIFT);
      end
      `JtagState_defaultEncoding_DR_EXIT1 : begin
        _zz_1_ = (io_jtag_tms ? `JtagState_defaultEncoding_DR_UPDATE : `JtagState_defaultEncoding_DR_PAUSE);
      end
      `JtagState_defaultEncoding_DR_PAUSE : begin
        _zz_1_ = (io_jtag_tms ? `JtagState_defaultEncoding_DR_EXIT2 : `JtagState_defaultEncoding_DR_PAUSE);
      end
      `JtagState_defaultEncoding_DR_EXIT2 : begin
        _zz_1_ = (io_jtag_tms ? `JtagState_defaultEncoding_DR_UPDATE : `JtagState_defaultEncoding_DR_SHIFT);
      end
      `JtagState_defaultEncoding_DR_UPDATE : begin
        _zz_1_ = (io_jtag_tms ? `JtagState_defaultEncoding_DR_SELECT : `JtagState_defaultEncoding_IDLE);
      end
      default : begin
        _zz_1_ = (io_jtag_tms ? `JtagState_defaultEncoding_RESET : `JtagState_defaultEncoding_IDLE);
      end
    endcase
  end

  assign jtag_tap_fsm_stateNext = _zz_1_;
  always @ (*) begin
    jtag_tap_tdoUnbufferd = jtag_tap_bypass;
    case(jtag_tap_fsm_state)
      `JtagState_defaultEncoding_IR_CAPTURE : begin
      end
      `JtagState_defaultEncoding_IR_SHIFT : begin
        jtag_tap_tdoUnbufferd = jtag_tap_instructionShift[0];
      end
      `JtagState_defaultEncoding_IR_UPDATE : begin
      end
      `JtagState_defaultEncoding_DR_SHIFT : begin
      end
      default : begin
      end
    endcase
    if(jtag_idcodeArea_instructionHit)begin
      if(_zz_2_)begin
        jtag_tap_tdoUnbufferd = jtag_idcodeArea_shifter[0];
      end
    end
    if(jtag_readArea_instructionHit)begin
      if(_zz_3_)begin
        jtag_tap_tdoUnbufferd = jtag_readArea_shifter[0];
      end
    end
  end

  assign io_jtag_tdo = jtag_tap_tdoUnbufferd_regNext;
  assign jtag_idcodeArea_instructionId = (1'b1);
  assign jtag_idcodeArea_instructionHit = (jtag_tap_instruction == _zz_4_);
  assign jtag_writeArea_instructionId = (2'b10);
  assign jtag_writeArea_instructionHit = (jtag_tap_instruction == _zz_5_);
  always @ (*) begin
    jtag_writeArea_source_valid = 1'b0;
    if(jtag_writeArea_instructionHit)begin
      if((jtag_tap_fsm_state == `JtagState_defaultEncoding_DR_SHIFT))begin
        jtag_writeArea_source_valid = 1'b1;
      end
    end
  end

  assign jtag_writeArea_source_payload_last = io_jtag_tms;
  assign jtag_writeArea_source_payload_fragment[0] = io_jtag_tdi;
  assign system_cmd_valid = flowCCByToggle_1__io_output_valid;
  assign system_cmd_payload_last = flowCCByToggle_1__io_output_payload_last;
  assign system_cmd_payload_fragment = flowCCByToggle_1__io_output_payload_fragment;
  assign jtag_readArea_instructionId = (2'b11);
  assign jtag_readArea_instructionHit = (jtag_tap_instruction == _zz_6_);
  always @ (posedge clk_12M) begin
    if(io_remote_cmd_valid)begin
      system_rsp_valid <= 1'b0;
    end
    if((io_remote_rsp_valid && io_remote_rsp_ready))begin
      system_rsp_valid <= 1'b1;
      system_rsp_payload_error <= io_remote_rsp_payload_error;
      system_rsp_payload_data <= io_remote_rsp_payload_data;
    end
  end

  always @ (posedge io_jtag_tck) begin
    jtag_tap_fsm_state <= jtag_tap_fsm_stateNext;
    case(jtag_tap_fsm_state)
      `JtagState_defaultEncoding_IR_CAPTURE : begin
        jtag_tap_instructionShift <= jtag_tap_instruction;
      end
      `JtagState_defaultEncoding_IR_SHIFT : begin
        jtag_tap_instructionShift <= ({io_jtag_tdi,jtag_tap_instructionShift} >>> 1);
      end
      `JtagState_defaultEncoding_IR_UPDATE : begin
        jtag_tap_instruction <= jtag_tap_instructionShift;
      end
      `JtagState_defaultEncoding_DR_SHIFT : begin
        jtag_tap_bypass <= io_jtag_tdi;
      end
      default : begin
      end
    endcase
    if(jtag_idcodeArea_instructionHit)begin
      if(_zz_2_)begin
        jtag_idcodeArea_shifter <= ({io_jtag_tdi,jtag_idcodeArea_shifter} >>> 1);
      end
    end
    if((jtag_tap_fsm_state == `JtagState_defaultEncoding_RESET))begin
      jtag_idcodeArea_shifter <= (32'b00010000000000000001111111111111);
      jtag_tap_instruction <= {3'd0, jtag_idcodeArea_instructionId};
    end
    if(jtag_readArea_instructionHit)begin
      if((jtag_tap_fsm_state == `JtagState_defaultEncoding_DR_CAPTURE))begin
        jtag_readArea_shifter <= {{system_rsp_payload_data,system_rsp_payload_error},system_rsp_valid};
      end
      if(_zz_3_)begin
        jtag_readArea_shifter <= ({io_jtag_tdi,jtag_readArea_shifter} >>> 1);
      end
    end
  end

  always @ (negedge io_jtag_tck) begin
    jtag_tap_tdoUnbufferd_regNext <= jtag_tap_tdoUnbufferd;
  end

endmodule

module SystemDebugger (
      input   io_remote_cmd_valid,
      output  io_remote_cmd_ready,
      input   io_remote_cmd_payload_last,
      input  [0:0] io_remote_cmd_payload_fragment,
      output  io_remote_rsp_valid,
      input   io_remote_rsp_ready,
      output  io_remote_rsp_payload_error,
      output [31:0] io_remote_rsp_payload_data,
      output  io_mem_cmd_valid,
      input   io_mem_cmd_ready,
      output [31:0] io_mem_cmd_payload_address,
      output [31:0] io_mem_cmd_payload_data,
      output  io_mem_cmd_payload_wr,
      output [1:0] io_mem_cmd_payload_size,
      input   io_mem_rsp_valid,
      input  [31:0] io_mem_rsp_payload,
      input   clk_12M,
      input   clockCtrl_resetUnbuffered_regNext);
  wire  _zz_2_;
  wire [0:0] _zz_3_;
  reg [66:0] dispatcher_dataShifter;
  reg  dispatcher_dataLoaded;
  reg [7:0] dispatcher_headerShifter;
  wire [7:0] dispatcher_header;
  reg  dispatcher_headerLoaded;
  reg [2:0] dispatcher_counter;
  wire [66:0] _zz_1_;
  assign _zz_2_ = (dispatcher_headerLoaded == 1'b0);
  assign _zz_3_ = _zz_1_[64 : 64];
  assign dispatcher_header = dispatcher_headerShifter[7 : 0];
  assign io_remote_cmd_ready = (! dispatcher_dataLoaded);
  assign _zz_1_ = dispatcher_dataShifter[66 : 0];
  assign io_mem_cmd_payload_address = _zz_1_[31 : 0];
  assign io_mem_cmd_payload_data = _zz_1_[63 : 32];
  assign io_mem_cmd_payload_wr = _zz_3_[0];
  assign io_mem_cmd_payload_size = _zz_1_[66 : 65];
  assign io_mem_cmd_valid = (dispatcher_dataLoaded && (dispatcher_header == (8'b00000000)));
  assign io_remote_rsp_valid = io_mem_rsp_valid;
  assign io_remote_rsp_payload_error = 1'b0;
  assign io_remote_rsp_payload_data = io_mem_rsp_payload;
  always @ (posedge clk_12M) begin
    if(clockCtrl_resetUnbuffered_regNext) begin
      dispatcher_dataLoaded <= 1'b0;
      dispatcher_headerLoaded <= 1'b0;
      dispatcher_counter <= (3'b000);
    end else begin
      if(io_remote_cmd_valid)begin
        if(_zz_2_)begin
          dispatcher_counter <= (dispatcher_counter + (3'b001));
          if((dispatcher_counter == (3'b111)))begin
            dispatcher_headerLoaded <= 1'b1;
          end
        end
        if(io_remote_cmd_payload_last)begin
          dispatcher_headerLoaded <= 1'b1;
          dispatcher_dataLoaded <= 1'b1;
          dispatcher_counter <= (3'b000);
        end
      end
      if((io_mem_cmd_valid && io_mem_cmd_ready))begin
        dispatcher_headerLoaded <= 1'b0;
        dispatcher_dataLoaded <= 1'b0;
      end
    end
  end

  always @ (posedge clk_12M) begin
    if(io_remote_cmd_valid)begin
      if(_zz_2_)begin
        dispatcher_headerShifter <= ({io_remote_cmd_payload_fragment,dispatcher_headerShifter} >>> 1);
      end else begin
        dispatcher_dataShifter <= ({io_remote_cmd_payload_fragment,dispatcher_dataShifter} >>> 1);
      end
    end
  end

endmodule

module BmbDecoder (
      input   io_input_cmd_valid,
      output  io_input_cmd_ready,
      input   io_input_cmd_payload_last,
      input  [0:0] io_input_cmd_payload_fragment_opcode,
      input  [31:0] io_input_cmd_payload_fragment_address,
      input  [4:0] io_input_cmd_payload_fragment_length,
      output  io_input_rsp_valid,
      input   io_input_rsp_ready,
      output  io_input_rsp_payload_last,
      output [0:0] io_input_rsp_payload_fragment_opcode,
      output [31:0] io_input_rsp_payload_fragment_data,
      output  io_outputs_0_cmd_valid,
      input   io_outputs_0_cmd_ready,
      output  io_outputs_0_cmd_payload_last,
      output [0:0] io_outputs_0_cmd_payload_fragment_opcode,
      output [31:0] io_outputs_0_cmd_payload_fragment_address,
      output [4:0] io_outputs_0_cmd_payload_fragment_length,
      input   io_outputs_0_rsp_valid,
      output  io_outputs_0_rsp_ready,
      input   io_outputs_0_rsp_payload_last,
      input  [0:0] io_outputs_0_rsp_payload_fragment_opcode,
      input  [31:0] io_outputs_0_rsp_payload_fragment_data);
  assign io_outputs_0_cmd_valid = io_input_cmd_valid;
  assign io_input_cmd_ready = io_outputs_0_cmd_ready;
  assign io_input_rsp_valid = io_outputs_0_rsp_valid;
  assign io_outputs_0_rsp_ready = io_input_rsp_ready;
  assign io_outputs_0_cmd_payload_last = io_input_cmd_payload_last;
  assign io_input_rsp_payload_last = io_outputs_0_rsp_payload_last;
  assign io_outputs_0_cmd_payload_fragment_opcode = io_input_cmd_payload_fragment_opcode;
  assign io_outputs_0_cmd_payload_fragment_address = io_input_cmd_payload_fragment_address;
  assign io_outputs_0_cmd_payload_fragment_length = io_input_cmd_payload_fragment_length;
  assign io_input_rsp_payload_fragment_opcode = io_outputs_0_rsp_payload_fragment_opcode;
  assign io_input_rsp_payload_fragment_data = io_outputs_0_rsp_payload_fragment_data;
endmodule

module BmbDecoder_1_ (
      input   io_input_cmd_valid,
      output  io_input_cmd_ready,
      input   io_input_cmd_payload_last,
      input  [0:0] io_input_cmd_payload_fragment_opcode,
      input  [31:0] io_input_cmd_payload_fragment_address,
      input  [1:0] io_input_cmd_payload_fragment_length,
      input  [31:0] io_input_cmd_payload_fragment_data,
      input  [3:0] io_input_cmd_payload_fragment_mask,
      input  [0:0] io_input_cmd_payload_fragment_context,
      output  io_input_rsp_valid,
      input   io_input_rsp_ready,
      output  io_input_rsp_payload_last,
      output [0:0] io_input_rsp_payload_fragment_opcode,
      output [31:0] io_input_rsp_payload_fragment_data,
      output [0:0] io_input_rsp_payload_fragment_context,
      output  io_outputs_0_cmd_valid,
      input   io_outputs_0_cmd_ready,
      output  io_outputs_0_cmd_payload_last,
      output [0:0] io_outputs_0_cmd_payload_fragment_opcode,
      output [31:0] io_outputs_0_cmd_payload_fragment_address,
      output [1:0] io_outputs_0_cmd_payload_fragment_length,
      output [31:0] io_outputs_0_cmd_payload_fragment_data,
      output [3:0] io_outputs_0_cmd_payload_fragment_mask,
      output [0:0] io_outputs_0_cmd_payload_fragment_context,
      input   io_outputs_0_rsp_valid,
      output  io_outputs_0_rsp_ready,
      input   io_outputs_0_rsp_payload_last,
      input  [0:0] io_outputs_0_rsp_payload_fragment_opcode,
      input  [31:0] io_outputs_0_rsp_payload_fragment_data,
      input  [0:0] io_outputs_0_rsp_payload_fragment_context);
  assign io_outputs_0_cmd_valid = io_input_cmd_valid;
  assign io_input_cmd_ready = io_outputs_0_cmd_ready;
  assign io_input_rsp_valid = io_outputs_0_rsp_valid;
  assign io_outputs_0_rsp_ready = io_input_rsp_ready;
  assign io_outputs_0_cmd_payload_last = io_input_cmd_payload_last;
  assign io_input_rsp_payload_last = io_outputs_0_rsp_payload_last;
  assign io_outputs_0_cmd_payload_fragment_opcode = io_input_cmd_payload_fragment_opcode;
  assign io_outputs_0_cmd_payload_fragment_address = io_input_cmd_payload_fragment_address;
  assign io_outputs_0_cmd_payload_fragment_length = io_input_cmd_payload_fragment_length;
  assign io_outputs_0_cmd_payload_fragment_data = io_input_cmd_payload_fragment_data;
  assign io_outputs_0_cmd_payload_fragment_mask = io_input_cmd_payload_fragment_mask;
  assign io_outputs_0_cmd_payload_fragment_context = io_input_cmd_payload_fragment_context;
  assign io_input_rsp_payload_fragment_opcode = io_outputs_0_rsp_payload_fragment_opcode;
  assign io_input_rsp_payload_fragment_data = io_outputs_0_rsp_payload_fragment_data;
  assign io_input_rsp_payload_fragment_context = io_outputs_0_rsp_payload_fragment_context;
endmodule

module Apb3SpiXdrMasterCtrl (
      input  [7:0] io_apb_PADDR,
      input  [0:0] io_apb_PSEL,
      input   io_apb_PENABLE,
      output  io_apb_PREADY,
      input   io_apb_PWRITE,
      input  [31:0] io_apb_PWDATA,
      output reg [31:0] io_apb_PRDATA,
      input   io_xip_cmd_valid,
      output  io_xip_cmd_ready,
      input  [23:0] io_xip_cmd_payload_address,
      input  [4:0] io_xip_cmd_payload_length,
      output  io_xip_rsp_valid,
      input   io_xip_rsp_ready,
      output  io_xip_rsp_payload_last,
      output [7:0] io_xip_rsp_payload_fragment,
      output [1:0] io_spi_sclk_write,
      output  io_spi_data_0_writeEnable,
      input  [1:0] io_spi_data_0_read,
      output [1:0] io_spi_data_0_write,
      output  io_spi_data_1_writeEnable,
      input  [1:0] io_spi_data_1_read,
      output [1:0] io_spi_data_1_write,
      output [0:0] io_spi_ss,
      output  io_interrupt,
      input   clk_12M,
      input   clockCtrl_systemReset);
  reg [0:0] _zz_20_;
  reg  _zz_21_;
  reg  _zz_22_;
  reg  _zz_23_;
  reg  _zz_24_;
  reg [7:0] _zz_25_;
  wire  _zz_26_;
  reg  _zz_27_;
  wire  _zz_28_;
  reg [7:0] _zz_29_;
  wire  ctrl_io_cmd_ready;
  wire  ctrl_io_rsp_valid;
  wire [7:0] ctrl_io_rsp_payload_data;
  wire [1:0] ctrl_io_spi_sclk_write;
  wire [0:0] ctrl_io_spi_ss;
  wire [1:0] ctrl_io_spi_data_0_write;
  wire  ctrl_io_spi_data_0_writeEnable;
  wire [1:0] ctrl_io_spi_data_1_write;
  wire  ctrl_io_spi_data_1_writeEnable;
  wire  mapping_cmdLogic_streamUnbuffered_queueWithAvailability_io_push_ready;
  wire  mapping_cmdLogic_streamUnbuffered_queueWithAvailability_io_pop_valid;
  wire  mapping_cmdLogic_streamUnbuffered_queueWithAvailability_io_pop_payload_kind;
  wire  mapping_cmdLogic_streamUnbuffered_queueWithAvailability_io_pop_payload_read;
  wire  mapping_cmdLogic_streamUnbuffered_queueWithAvailability_io_pop_payload_write;
  wire [7:0] mapping_cmdLogic_streamUnbuffered_queueWithAvailability_io_pop_payload_data;
  wire [6:0] mapping_cmdLogic_streamUnbuffered_queueWithAvailability_io_occupancy;
  wire [6:0] mapping_cmdLogic_streamUnbuffered_queueWithAvailability_io_availability;
  wire  ctrl_io_rsp_queueWithOccupancy_io_push_ready;
  wire  ctrl_io_rsp_queueWithOccupancy_io_pop_valid;
  wire [7:0] ctrl_io_rsp_queueWithOccupancy_io_pop_payload_data;
  wire [6:0] ctrl_io_rsp_queueWithOccupancy_io_occupancy;
  wire [6:0] ctrl_io_rsp_queueWithOccupancy_io_availability;
  wire  _zz_30_;
  wire  _zz_31_;
  wire [0:0] _zz_32_;
  wire [0:0] _zz_33_;
  wire [0:0] _zz_34_;
  wire [0:0] _zz_35_;
  wire [0:0] _zz_36_;
  wire [0:0] _zz_37_;
  wire [0:0] _zz_38_;
  wire [0:0] _zz_39_;
  wire [4:0] _zz_40_;
  wire [1:0] _zz_41_;
  wire  _zz_1_;
  wire  _zz_2_;
  wire  mapping_cmdLogic_streamUnbuffered_valid;
  wire  mapping_cmdLogic_streamUnbuffered_ready;
  wire  mapping_cmdLogic_streamUnbuffered_payload_kind;
  wire  mapping_cmdLogic_streamUnbuffered_payload_read;
  wire  mapping_cmdLogic_streamUnbuffered_payload_write;
  wire [7:0] mapping_cmdLogic_streamUnbuffered_payload_data;
  reg  _zz_3_;
  reg  mapping_interruptCtrl_cmdIntEnable;
  reg  mapping_interruptCtrl_rspIntEnable;
  wire  mapping_interruptCtrl_cmdInt;
  wire  mapping_interruptCtrl_rspInt;
  wire  mapping_interruptCtrl_interrupt;
  reg  _zz_4_;
  reg  _zz_5_;
  reg [0:0] _zz_6_;
  reg [7:0] _zz_7_;
  reg [7:0] _zz_8_;
  reg [7:0] _zz_9_;
  reg [7:0] _zz_10_;
  reg [0:0] _zz_11_;
  wire  mapping_xip_xipBus_cmd_valid;
  wire  mapping_xip_xipBus_cmd_ready;
  wire [23:0] mapping_xip_xipBus_cmd_payload_address;
  wire [4:0] mapping_xip_xipBus_cmd_payload_length;
  reg  mapping_xip_xipBus_rsp_valid;
  wire  mapping_xip_xipBus_rsp_ready;
  wire  mapping_xip_xipBus_rsp_payload_last;
  wire [7:0] mapping_xip_xipBus_rsp_payload_fragment;
  reg  mapping_xip_enable;
  wire [0:0] mapping_xip_instructionMod;
  wire  mapping_xip_instructionEnable;
  wire [7:0] mapping_xip_instructionData;
  wire [0:0] mapping_xip_addressMod;
  wire [3:0] mapping_xip_dummyCount;
  wire [7:0] mapping_xip_dummyData;
  wire [0:0] mapping_xip_dummyMod;
  wire [0:0] mapping_xip_payloadMod;
  wire  mapping_xip_fsm_wantExit;
  reg [4:0] mapping_xip_fsm_cmdLength;
  reg [4:0] mapping_xip_fsm_rspCounter;
  wire  mapping_xip_fsm_rspCounterMatch;
  reg  mapping_xip_fsm_cmdHalt;
  reg  mapping_xip_fsm_xipToCtrlCmd_valid;
  wire  mapping_xip_fsm_xipToCtrlCmd_ready;
  reg  mapping_xip_fsm_xipToCtrlCmd_payload_kind;
  reg  mapping_xip_fsm_xipToCtrlCmd_payload_read;
  reg  mapping_xip_fsm_xipToCtrlCmd_payload_write;
  reg [7:0] mapping_xip_fsm_xipToCtrlCmd_payload_data;
  reg [0:0] mapping_xip_fsm_xipToCtrlMod;
  wire  _zz_12_;
  wire  _zz_13_;
  wire  mapping_xip_fsm_xipToCtrlCmdBuffer_valid;
  wire  mapping_xip_fsm_xipToCtrlCmdBuffer_ready;
  wire  mapping_xip_fsm_xipToCtrlCmdBuffer_payload_kind;
  wire  mapping_xip_fsm_xipToCtrlCmdBuffer_payload_read;
  wire  mapping_xip_fsm_xipToCtrlCmdBuffer_payload_write;
  wire [7:0] mapping_xip_fsm_xipToCtrlCmdBuffer_payload_data;
  reg  _zz_14_;
  reg  _zz_15_;
  reg  _zz_16_;
  reg  _zz_17_;
  reg [7:0] _zz_18_;
  reg [0:0] mapping_xip_fsm_xipToCtrlModBuffer;
  reg  mapping_xip_fsm_xipBusCmdReadyReg;
  reg [4:0] mapping_xip_fsm_counter;
  reg  mapping_xip_fsm_lastFired;
  wire [1:0] _zz_19_;
  reg `mapping_xip_fsm_enumDefinition_defaultEncoding_type mapping_xip_fsm_stateReg;
  reg `mapping_xip_fsm_enumDefinition_defaultEncoding_type mapping_xip_fsm_stateNext;
  `ifndef SYNTHESIS
  reg [215:0] mapping_xip_fsm_stateReg_string;
  reg [215:0] mapping_xip_fsm_stateNext_string;
  `endif

  assign _zz_30_ = (! (mapping_xip_fsm_stateReg == `mapping_xip_fsm_enumDefinition_defaultEncoding_mapping_xip_fsm_IDLE));
  assign _zz_31_ = (mapping_xip_fsm_counter == (5'b00010));
  assign _zz_32_ = io_apb_PWDATA[8 : 8];
  assign _zz_33_ = io_apb_PWDATA[9 : 9];
  assign _zz_34_ = io_apb_PWDATA[11 : 11];
  assign _zz_35_ = io_apb_PWDATA[0 : 0];
  assign _zz_36_ = io_apb_PWDATA[1 : 1];
  assign _zz_37_ = _zz_19_[0 : 0];
  assign _zz_38_ = _zz_19_[1 : 1];
  assign _zz_39_ = io_apb_PWDATA[0 : 0];
  assign _zz_40_ = {1'd0, mapping_xip_dummyCount};
  assign _zz_41_ = mapping_xip_fsm_counter[1 : 0];
  SpiXdrMasterCtrl ctrl ( 
    .io_config_kind_cpol(_zz_4_),
    .io_config_kind_cpha(_zz_5_),
    .io_config_sclkToogle(_zz_7_),
    .io_config_mod(_zz_20_),
    .io_config_ss_activeHigh(_zz_11_),
    .io_config_ss_setup(_zz_8_),
    .io_config_ss_hold(_zz_9_),
    .io_config_ss_disable(_zz_10_),
    .io_cmd_valid(_zz_21_),
    .io_cmd_ready(ctrl_io_cmd_ready),
    .io_cmd_payload_kind(_zz_22_),
    .io_cmd_payload_read(_zz_23_),
    .io_cmd_payload_write(_zz_24_),
    .io_cmd_payload_data(_zz_25_),
    .io_rsp_valid(ctrl_io_rsp_valid),
    .io_rsp_payload_data(ctrl_io_rsp_payload_data),
    .io_spi_sclk_write(ctrl_io_spi_sclk_write),
    .io_spi_data_0_writeEnable(ctrl_io_spi_data_0_writeEnable),
    .io_spi_data_0_read(io_spi_data_0_read),
    .io_spi_data_0_write(ctrl_io_spi_data_0_write),
    .io_spi_data_1_writeEnable(ctrl_io_spi_data_1_writeEnable),
    .io_spi_data_1_read(io_spi_data_1_read),
    .io_spi_data_1_write(ctrl_io_spi_data_1_write),
    .io_spi_ss(ctrl_io_spi_ss),
    .clk_12M(clk_12M),
    .clockCtrl_systemReset(clockCtrl_systemReset) 
  );
  StreamFifo_2_ mapping_cmdLogic_streamUnbuffered_queueWithAvailability ( 
    .io_push_valid(mapping_cmdLogic_streamUnbuffered_valid),
    .io_push_ready(mapping_cmdLogic_streamUnbuffered_queueWithAvailability_io_push_ready),
    .io_push_payload_kind(mapping_cmdLogic_streamUnbuffered_payload_kind),
    .io_push_payload_read(mapping_cmdLogic_streamUnbuffered_payload_read),
    .io_push_payload_write(mapping_cmdLogic_streamUnbuffered_payload_write),
    .io_push_payload_data(mapping_cmdLogic_streamUnbuffered_payload_data),
    .io_pop_valid(mapping_cmdLogic_streamUnbuffered_queueWithAvailability_io_pop_valid),
    .io_pop_ready(ctrl_io_cmd_ready),
    .io_pop_payload_kind(mapping_cmdLogic_streamUnbuffered_queueWithAvailability_io_pop_payload_kind),
    .io_pop_payload_read(mapping_cmdLogic_streamUnbuffered_queueWithAvailability_io_pop_payload_read),
    .io_pop_payload_write(mapping_cmdLogic_streamUnbuffered_queueWithAvailability_io_pop_payload_write),
    .io_pop_payload_data(mapping_cmdLogic_streamUnbuffered_queueWithAvailability_io_pop_payload_data),
    .io_flush(_zz_26_),
    .io_occupancy(mapping_cmdLogic_streamUnbuffered_queueWithAvailability_io_occupancy),
    .io_availability(mapping_cmdLogic_streamUnbuffered_queueWithAvailability_io_availability),
    .clk_12M(clk_12M),
    .clockCtrl_systemReset(clockCtrl_systemReset) 
  );
  StreamFifo_3_ ctrl_io_rsp_queueWithOccupancy ( 
    .io_push_valid(ctrl_io_rsp_valid),
    .io_push_ready(ctrl_io_rsp_queueWithOccupancy_io_push_ready),
    .io_push_payload_data(ctrl_io_rsp_payload_data),
    .io_pop_valid(ctrl_io_rsp_queueWithOccupancy_io_pop_valid),
    .io_pop_ready(_zz_27_),
    .io_pop_payload_data(ctrl_io_rsp_queueWithOccupancy_io_pop_payload_data),
    .io_flush(_zz_28_),
    .io_occupancy(ctrl_io_rsp_queueWithOccupancy_io_occupancy),
    .io_availability(ctrl_io_rsp_queueWithOccupancy_io_availability),
    .clk_12M(clk_12M),
    .clockCtrl_systemReset(clockCtrl_systemReset) 
  );
  always @(*) begin
    case(_zz_41_)
      2'b00 : begin
        _zz_29_ = mapping_xip_xipBus_cmd_payload_address[23 : 16];
      end
      2'b01 : begin
        _zz_29_ = mapping_xip_xipBus_cmd_payload_address[15 : 8];
      end
      default : begin
        _zz_29_ = mapping_xip_xipBus_cmd_payload_address[7 : 0];
      end
    endcase
  end

  `ifndef SYNTHESIS
  always @(*) begin
    case(mapping_xip_fsm_stateReg)
      `mapping_xip_fsm_enumDefinition_defaultEncoding_boot : mapping_xip_fsm_stateReg_string = "boot                       ";
      `mapping_xip_fsm_enumDefinition_defaultEncoding_mapping_xip_fsm_IDLE : mapping_xip_fsm_stateReg_string = "mapping_xip_fsm_IDLE       ";
      `mapping_xip_fsm_enumDefinition_defaultEncoding_mapping_xip_fsm_INSTRUCTION : mapping_xip_fsm_stateReg_string = "mapping_xip_fsm_INSTRUCTION";
      `mapping_xip_fsm_enumDefinition_defaultEncoding_mapping_xip_fsm_ADDRESS : mapping_xip_fsm_stateReg_string = "mapping_xip_fsm_ADDRESS    ";
      `mapping_xip_fsm_enumDefinition_defaultEncoding_mapping_xip_fsm_DUMMY : mapping_xip_fsm_stateReg_string = "mapping_xip_fsm_DUMMY      ";
      `mapping_xip_fsm_enumDefinition_defaultEncoding_mapping_xip_fsm_PAYLOAD : mapping_xip_fsm_stateReg_string = "mapping_xip_fsm_PAYLOAD    ";
      `mapping_xip_fsm_enumDefinition_defaultEncoding_mapping_xip_fsm_STOP : mapping_xip_fsm_stateReg_string = "mapping_xip_fsm_STOP       ";
      default : mapping_xip_fsm_stateReg_string = "???????????????????????????";
    endcase
  end
  always @(*) begin
    case(mapping_xip_fsm_stateNext)
      `mapping_xip_fsm_enumDefinition_defaultEncoding_boot : mapping_xip_fsm_stateNext_string = "boot                       ";
      `mapping_xip_fsm_enumDefinition_defaultEncoding_mapping_xip_fsm_IDLE : mapping_xip_fsm_stateNext_string = "mapping_xip_fsm_IDLE       ";
      `mapping_xip_fsm_enumDefinition_defaultEncoding_mapping_xip_fsm_INSTRUCTION : mapping_xip_fsm_stateNext_string = "mapping_xip_fsm_INSTRUCTION";
      `mapping_xip_fsm_enumDefinition_defaultEncoding_mapping_xip_fsm_ADDRESS : mapping_xip_fsm_stateNext_string = "mapping_xip_fsm_ADDRESS    ";
      `mapping_xip_fsm_enumDefinition_defaultEncoding_mapping_xip_fsm_DUMMY : mapping_xip_fsm_stateNext_string = "mapping_xip_fsm_DUMMY      ";
      `mapping_xip_fsm_enumDefinition_defaultEncoding_mapping_xip_fsm_PAYLOAD : mapping_xip_fsm_stateNext_string = "mapping_xip_fsm_PAYLOAD    ";
      `mapping_xip_fsm_enumDefinition_defaultEncoding_mapping_xip_fsm_STOP : mapping_xip_fsm_stateNext_string = "mapping_xip_fsm_STOP       ";
      default : mapping_xip_fsm_stateNext_string = "???????????????????????????";
    endcase
  end
  `endif

  assign io_apb_PREADY = 1'b1;
  always @ (*) begin
    io_apb_PRDATA = (32'b00000000000000000000000000000000);
    case(io_apb_PADDR)
      8'b00000000 : begin
        io_apb_PRDATA[31 : 31] = (ctrl_io_rsp_queueWithOccupancy_io_pop_valid ^ 1'b1);
        io_apb_PRDATA[7 : 0] = ctrl_io_rsp_queueWithOccupancy_io_pop_payload_data;
      end
      8'b00000100 : begin
        io_apb_PRDATA[6 : 0] = mapping_cmdLogic_streamUnbuffered_queueWithAvailability_io_availability;
        io_apb_PRDATA[22 : 16] = ctrl_io_rsp_queueWithOccupancy_io_occupancy;
      end
      8'b00001100 : begin
        io_apb_PRDATA[0 : 0] = mapping_interruptCtrl_cmdIntEnable;
        io_apb_PRDATA[1 : 1] = mapping_interruptCtrl_rspIntEnable;
        io_apb_PRDATA[8 : 8] = mapping_interruptCtrl_cmdInt;
        io_apb_PRDATA[9 : 9] = mapping_interruptCtrl_rspInt;
      end
      8'b00001000 : begin
      end
      8'b00100000 : begin
      end
      8'b00100100 : begin
      end
      8'b00101000 : begin
      end
      8'b00101100 : begin
      end
      8'b00110000 : begin
      end
      8'b01000000 : begin
      end
      default : begin
      end
    endcase
  end

  assign _zz_1_ = (((io_apb_PSEL[0] && io_apb_PENABLE) && io_apb_PREADY) && io_apb_PWRITE);
  assign _zz_2_ = (((io_apb_PSEL[0] && io_apb_PENABLE) && io_apb_PREADY) && (! io_apb_PWRITE));
  always @ (*) begin
    _zz_3_ = 1'b0;
    case(io_apb_PADDR)
      8'b00000000 : begin
        if(_zz_1_)begin
          _zz_3_ = 1'b1;
        end
      end
      8'b00000100 : begin
      end
      8'b00001100 : begin
      end
      8'b00001000 : begin
      end
      8'b00100000 : begin
      end
      8'b00100100 : begin
      end
      8'b00101000 : begin
      end
      8'b00101100 : begin
      end
      8'b00110000 : begin
      end
      8'b01000000 : begin
      end
      default : begin
      end
    endcase
  end

  assign mapping_cmdLogic_streamUnbuffered_valid = _zz_3_;
  assign mapping_cmdLogic_streamUnbuffered_ready = mapping_cmdLogic_streamUnbuffered_queueWithAvailability_io_push_ready;
  always @ (*) begin
    _zz_21_ = mapping_cmdLogic_streamUnbuffered_queueWithAvailability_io_pop_valid;
    if(mapping_xip_fsm_xipToCtrlCmdBuffer_valid)begin
      _zz_21_ = 1'b1;
    end
  end

  always @ (*) begin
    _zz_22_ = mapping_cmdLogic_streamUnbuffered_queueWithAvailability_io_pop_payload_kind;
    if(mapping_xip_fsm_xipToCtrlCmdBuffer_valid)begin
      _zz_22_ = mapping_xip_fsm_xipToCtrlCmdBuffer_payload_kind;
    end
  end

  always @ (*) begin
    _zz_23_ = mapping_cmdLogic_streamUnbuffered_queueWithAvailability_io_pop_payload_read;
    if(mapping_xip_fsm_xipToCtrlCmdBuffer_valid)begin
      _zz_23_ = mapping_xip_fsm_xipToCtrlCmdBuffer_payload_read;
    end
  end

  always @ (*) begin
    _zz_24_ = mapping_cmdLogic_streamUnbuffered_queueWithAvailability_io_pop_payload_write;
    if(mapping_xip_fsm_xipToCtrlCmdBuffer_valid)begin
      _zz_24_ = mapping_xip_fsm_xipToCtrlCmdBuffer_payload_write;
    end
  end

  always @ (*) begin
    _zz_25_ = mapping_cmdLogic_streamUnbuffered_queueWithAvailability_io_pop_payload_data;
    if(mapping_xip_fsm_xipToCtrlCmdBuffer_valid)begin
      _zz_25_ = mapping_xip_fsm_xipToCtrlCmdBuffer_payload_data;
    end
  end

  always @ (*) begin
    _zz_27_ = 1'b0;
    if(mapping_xip_fsm_cmdHalt)begin
      _zz_27_ = 1'b0;
    end
    case(io_apb_PADDR)
      8'b00000000 : begin
        if(_zz_2_)begin
          _zz_27_ = 1'b1;
        end
      end
      8'b00000100 : begin
      end
      8'b00001100 : begin
      end
      8'b00001000 : begin
      end
      8'b00100000 : begin
      end
      8'b00100100 : begin
      end
      8'b00101000 : begin
      end
      8'b00101100 : begin
      end
      8'b00110000 : begin
      end
      8'b01000000 : begin
      end
      default : begin
      end
    endcase
    if(_zz_30_)begin
      if(mapping_xip_xipBus_rsp_ready)begin
        _zz_27_ = 1'b1;
      end
    end
  end

  assign mapping_interruptCtrl_cmdInt = (mapping_interruptCtrl_cmdIntEnable && (! mapping_cmdLogic_streamUnbuffered_queueWithAvailability_io_pop_valid));
  assign mapping_interruptCtrl_rspInt = (mapping_interruptCtrl_rspIntEnable && ctrl_io_rsp_queueWithOccupancy_io_pop_valid);
  assign mapping_interruptCtrl_interrupt = (mapping_interruptCtrl_rspInt || mapping_interruptCtrl_cmdInt);
  always @ (*) begin
    _zz_20_ = _zz_6_;
    if(mapping_xip_fsm_xipToCtrlCmdBuffer_valid)begin
      _zz_20_ = mapping_xip_fsm_xipToCtrlModBuffer;
    end
  end

  assign mapping_xip_instructionMod = (1'b0);
  assign mapping_xip_addressMod = (1'b0);
  assign mapping_xip_dummyMod = (1'b0);
  assign mapping_xip_payloadMod = (1'b1);
  assign mapping_xip_instructionEnable = 1'b1;
  assign mapping_xip_instructionData = (8'b00111011);
  assign mapping_xip_dummyCount = (4'b0000);
  assign mapping_xip_dummyData = (8'b11111111);
  assign mapping_xip_fsm_wantExit = 1'b0;
  assign mapping_xip_fsm_rspCounterMatch = (mapping_xip_fsm_rspCounter == mapping_xip_fsm_cmdLength);
  always @ (*) begin
    mapping_xip_xipBus_rsp_valid = 1'b0;
    if(_zz_30_)begin
      if(ctrl_io_rsp_queueWithOccupancy_io_pop_valid)begin
        mapping_xip_xipBus_rsp_valid = 1'b1;
      end
    end
  end

  assign mapping_xip_xipBus_rsp_payload_fragment = ctrl_io_rsp_queueWithOccupancy_io_pop_payload_data;
  assign mapping_xip_xipBus_rsp_payload_last = mapping_xip_fsm_rspCounterMatch;
  always @ (*) begin
    mapping_xip_fsm_cmdHalt = 1'b0;
    if(_zz_30_)begin
      if((mapping_xip_xipBus_rsp_valid && (! mapping_xip_xipBus_rsp_ready)))begin
        mapping_xip_fsm_cmdHalt = 1'b1;
      end
    end
  end

  always @ (*) begin
    mapping_xip_fsm_xipToCtrlMod = (1'bx);
    case(mapping_xip_fsm_stateReg)
      `mapping_xip_fsm_enumDefinition_defaultEncoding_mapping_xip_fsm_IDLE : begin
      end
      `mapping_xip_fsm_enumDefinition_defaultEncoding_mapping_xip_fsm_INSTRUCTION : begin
        mapping_xip_fsm_xipToCtrlMod = mapping_xip_instructionMod;
      end
      `mapping_xip_fsm_enumDefinition_defaultEncoding_mapping_xip_fsm_ADDRESS : begin
        mapping_xip_fsm_xipToCtrlMod = mapping_xip_addressMod;
      end
      `mapping_xip_fsm_enumDefinition_defaultEncoding_mapping_xip_fsm_DUMMY : begin
        mapping_xip_fsm_xipToCtrlMod = mapping_xip_dummyMod;
      end
      `mapping_xip_fsm_enumDefinition_defaultEncoding_mapping_xip_fsm_PAYLOAD : begin
        mapping_xip_fsm_xipToCtrlMod = mapping_xip_payloadMod;
      end
      `mapping_xip_fsm_enumDefinition_defaultEncoding_mapping_xip_fsm_STOP : begin
        mapping_xip_fsm_xipToCtrlMod = mapping_xip_payloadMod;
      end
      default : begin
      end
    endcase
  end

  always @ (*) begin
    mapping_xip_fsm_xipToCtrlCmd_valid = 1'b0;
    case(mapping_xip_fsm_stateReg)
      `mapping_xip_fsm_enumDefinition_defaultEncoding_mapping_xip_fsm_IDLE : begin
        if(mapping_xip_xipBus_cmd_valid)begin
          mapping_xip_fsm_xipToCtrlCmd_valid = 1'b1;
        end
      end
      `mapping_xip_fsm_enumDefinition_defaultEncoding_mapping_xip_fsm_INSTRUCTION : begin
        mapping_xip_fsm_xipToCtrlCmd_valid = 1'b1;
      end
      `mapping_xip_fsm_enumDefinition_defaultEncoding_mapping_xip_fsm_ADDRESS : begin
        mapping_xip_fsm_xipToCtrlCmd_valid = 1'b1;
      end
      `mapping_xip_fsm_enumDefinition_defaultEncoding_mapping_xip_fsm_DUMMY : begin
        mapping_xip_fsm_xipToCtrlCmd_valid = 1'b1;
      end
      `mapping_xip_fsm_enumDefinition_defaultEncoding_mapping_xip_fsm_PAYLOAD : begin
        mapping_xip_fsm_xipToCtrlCmd_valid = 1'b1;
      end
      `mapping_xip_fsm_enumDefinition_defaultEncoding_mapping_xip_fsm_STOP : begin
        if(mapping_xip_fsm_lastFired)begin
          mapping_xip_fsm_xipToCtrlCmd_valid = 1'b1;
        end
      end
      default : begin
      end
    endcase
  end

  always @ (*) begin
    mapping_xip_fsm_xipToCtrlCmd_payload_kind = 1'bx;
    case(mapping_xip_fsm_stateReg)
      `mapping_xip_fsm_enumDefinition_defaultEncoding_mapping_xip_fsm_IDLE : begin
        if(mapping_xip_xipBus_cmd_valid)begin
          mapping_xip_fsm_xipToCtrlCmd_payload_kind = 1'b1;
        end
      end
      `mapping_xip_fsm_enumDefinition_defaultEncoding_mapping_xip_fsm_INSTRUCTION : begin
        mapping_xip_fsm_xipToCtrlCmd_payload_kind = 1'b0;
      end
      `mapping_xip_fsm_enumDefinition_defaultEncoding_mapping_xip_fsm_ADDRESS : begin
        mapping_xip_fsm_xipToCtrlCmd_payload_kind = 1'b0;
      end
      `mapping_xip_fsm_enumDefinition_defaultEncoding_mapping_xip_fsm_DUMMY : begin
        mapping_xip_fsm_xipToCtrlCmd_payload_kind = 1'b0;
      end
      `mapping_xip_fsm_enumDefinition_defaultEncoding_mapping_xip_fsm_PAYLOAD : begin
        mapping_xip_fsm_xipToCtrlCmd_payload_kind = 1'b0;
      end
      `mapping_xip_fsm_enumDefinition_defaultEncoding_mapping_xip_fsm_STOP : begin
        mapping_xip_fsm_xipToCtrlCmd_payload_kind = 1'b1;
      end
      default : begin
      end
    endcase
  end

  always @ (*) begin
    mapping_xip_fsm_xipToCtrlCmd_payload_read = 1'bx;
    case(mapping_xip_fsm_stateReg)
      `mapping_xip_fsm_enumDefinition_defaultEncoding_mapping_xip_fsm_IDLE : begin
      end
      `mapping_xip_fsm_enumDefinition_defaultEncoding_mapping_xip_fsm_INSTRUCTION : begin
        mapping_xip_fsm_xipToCtrlCmd_payload_read = 1'b0;
      end
      `mapping_xip_fsm_enumDefinition_defaultEncoding_mapping_xip_fsm_ADDRESS : begin
        mapping_xip_fsm_xipToCtrlCmd_payload_read = 1'b0;
      end
      `mapping_xip_fsm_enumDefinition_defaultEncoding_mapping_xip_fsm_DUMMY : begin
        mapping_xip_fsm_xipToCtrlCmd_payload_read = 1'b0;
      end
      `mapping_xip_fsm_enumDefinition_defaultEncoding_mapping_xip_fsm_PAYLOAD : begin
        mapping_xip_fsm_xipToCtrlCmd_payload_read = 1'b1;
      end
      `mapping_xip_fsm_enumDefinition_defaultEncoding_mapping_xip_fsm_STOP : begin
      end
      default : begin
      end
    endcase
  end

  always @ (*) begin
    mapping_xip_fsm_xipToCtrlCmd_payload_write = 1'bx;
    case(mapping_xip_fsm_stateReg)
      `mapping_xip_fsm_enumDefinition_defaultEncoding_mapping_xip_fsm_IDLE : begin
      end
      `mapping_xip_fsm_enumDefinition_defaultEncoding_mapping_xip_fsm_INSTRUCTION : begin
        mapping_xip_fsm_xipToCtrlCmd_payload_write = 1'b1;
      end
      `mapping_xip_fsm_enumDefinition_defaultEncoding_mapping_xip_fsm_ADDRESS : begin
        mapping_xip_fsm_xipToCtrlCmd_payload_write = 1'b1;
      end
      `mapping_xip_fsm_enumDefinition_defaultEncoding_mapping_xip_fsm_DUMMY : begin
        mapping_xip_fsm_xipToCtrlCmd_payload_write = 1'b1;
      end
      `mapping_xip_fsm_enumDefinition_defaultEncoding_mapping_xip_fsm_PAYLOAD : begin
        mapping_xip_fsm_xipToCtrlCmd_payload_write = 1'b0;
      end
      `mapping_xip_fsm_enumDefinition_defaultEncoding_mapping_xip_fsm_STOP : begin
      end
      default : begin
      end
    endcase
  end

  always @ (*) begin
    mapping_xip_fsm_xipToCtrlCmd_payload_data = (8'bxxxxxxxx);
    case(mapping_xip_fsm_stateReg)
      `mapping_xip_fsm_enumDefinition_defaultEncoding_mapping_xip_fsm_IDLE : begin
        if(mapping_xip_xipBus_cmd_valid)begin
          mapping_xip_fsm_xipToCtrlCmd_payload_data = (8'b10000000);
        end
      end
      `mapping_xip_fsm_enumDefinition_defaultEncoding_mapping_xip_fsm_INSTRUCTION : begin
        mapping_xip_fsm_xipToCtrlCmd_payload_data = mapping_xip_instructionData;
      end
      `mapping_xip_fsm_enumDefinition_defaultEncoding_mapping_xip_fsm_ADDRESS : begin
        mapping_xip_fsm_xipToCtrlCmd_payload_data = _zz_29_;
      end
      `mapping_xip_fsm_enumDefinition_defaultEncoding_mapping_xip_fsm_DUMMY : begin
        mapping_xip_fsm_xipToCtrlCmd_payload_data = mapping_xip_dummyData;
      end
      `mapping_xip_fsm_enumDefinition_defaultEncoding_mapping_xip_fsm_PAYLOAD : begin
      end
      `mapping_xip_fsm_enumDefinition_defaultEncoding_mapping_xip_fsm_STOP : begin
        mapping_xip_fsm_xipToCtrlCmd_payload_data = (8'b00000000);
      end
      default : begin
      end
    endcase
  end

  assign _zz_12_ = (! mapping_xip_fsm_cmdHalt);
  assign mapping_xip_fsm_xipToCtrlCmd_ready = (_zz_13_ && _zz_12_);
  assign _zz_13_ = ((1'b1 && (! mapping_xip_fsm_xipToCtrlCmdBuffer_valid)) || mapping_xip_fsm_xipToCtrlCmdBuffer_ready);
  assign mapping_xip_fsm_xipToCtrlCmdBuffer_valid = _zz_14_;
  assign mapping_xip_fsm_xipToCtrlCmdBuffer_payload_kind = _zz_15_;
  assign mapping_xip_fsm_xipToCtrlCmdBuffer_payload_read = _zz_16_;
  assign mapping_xip_fsm_xipToCtrlCmdBuffer_payload_write = _zz_17_;
  assign mapping_xip_fsm_xipToCtrlCmdBuffer_payload_data = _zz_18_;
  assign mapping_xip_fsm_xipToCtrlCmdBuffer_ready = ctrl_io_cmd_ready;
  assign mapping_xip_xipBus_cmd_ready = mapping_xip_fsm_xipBusCmdReadyReg;
  assign mapping_xip_xipBus_cmd_valid = io_xip_cmd_valid;
  assign io_xip_cmd_ready = mapping_xip_xipBus_cmd_ready;
  assign mapping_xip_xipBus_cmd_payload_address = io_xip_cmd_payload_address;
  assign mapping_xip_xipBus_cmd_payload_length = io_xip_cmd_payload_length;
  assign io_xip_rsp_valid = mapping_xip_xipBus_rsp_valid;
  assign mapping_xip_xipBus_rsp_ready = io_xip_rsp_ready;
  assign io_xip_rsp_payload_last = mapping_xip_xipBus_rsp_payload_last;
  assign io_xip_rsp_payload_fragment = mapping_xip_xipBus_rsp_payload_fragment;
  assign io_spi_sclk_write = ctrl_io_spi_sclk_write;
  assign io_spi_data_0_writeEnable = ctrl_io_spi_data_0_writeEnable;
  assign io_spi_data_0_write = ctrl_io_spi_data_0_write;
  assign io_spi_data_1_writeEnable = ctrl_io_spi_data_1_writeEnable;
  assign io_spi_data_1_write = ctrl_io_spi_data_1_write;
  assign io_spi_ss = ctrl_io_spi_ss;
  assign io_interrupt = mapping_interruptCtrl_interrupt;
  assign mapping_cmdLogic_streamUnbuffered_payload_data = io_apb_PWDATA[7 : 0];
  assign mapping_cmdLogic_streamUnbuffered_payload_write = _zz_32_[0];
  assign mapping_cmdLogic_streamUnbuffered_payload_read = _zz_33_[0];
  assign mapping_cmdLogic_streamUnbuffered_payload_kind = _zz_34_[0];
  assign _zz_19_ = io_apb_PWDATA[1 : 0];
  always @ (*) begin
    mapping_xip_fsm_stateNext = mapping_xip_fsm_stateReg;
    case(mapping_xip_fsm_stateReg)
      `mapping_xip_fsm_enumDefinition_defaultEncoding_mapping_xip_fsm_IDLE : begin
        if(mapping_xip_xipBus_cmd_valid)begin
          if(mapping_xip_fsm_xipToCtrlCmd_ready)begin
            if(mapping_xip_instructionEnable)begin
              mapping_xip_fsm_stateNext = `mapping_xip_fsm_enumDefinition_defaultEncoding_mapping_xip_fsm_INSTRUCTION;
            end else begin
              mapping_xip_fsm_stateNext = `mapping_xip_fsm_enumDefinition_defaultEncoding_mapping_xip_fsm_ADDRESS;
            end
          end
        end
      end
      `mapping_xip_fsm_enumDefinition_defaultEncoding_mapping_xip_fsm_INSTRUCTION : begin
        if(mapping_xip_fsm_xipToCtrlCmd_ready)begin
          mapping_xip_fsm_stateNext = `mapping_xip_fsm_enumDefinition_defaultEncoding_mapping_xip_fsm_ADDRESS;
        end
      end
      `mapping_xip_fsm_enumDefinition_defaultEncoding_mapping_xip_fsm_ADDRESS : begin
        if(mapping_xip_fsm_xipToCtrlCmd_ready)begin
          if(_zz_31_)begin
            mapping_xip_fsm_stateNext = `mapping_xip_fsm_enumDefinition_defaultEncoding_mapping_xip_fsm_DUMMY;
          end
        end
      end
      `mapping_xip_fsm_enumDefinition_defaultEncoding_mapping_xip_fsm_DUMMY : begin
        if(mapping_xip_fsm_xipToCtrlCmd_ready)begin
          if((mapping_xip_fsm_counter == _zz_40_))begin
            mapping_xip_fsm_stateNext = `mapping_xip_fsm_enumDefinition_defaultEncoding_mapping_xip_fsm_PAYLOAD;
          end
        end
      end
      `mapping_xip_fsm_enumDefinition_defaultEncoding_mapping_xip_fsm_PAYLOAD : begin
        if(mapping_xip_fsm_xipToCtrlCmd_ready)begin
          if((mapping_xip_fsm_counter == mapping_xip_fsm_cmdLength))begin
            mapping_xip_fsm_stateNext = `mapping_xip_fsm_enumDefinition_defaultEncoding_mapping_xip_fsm_STOP;
          end
        end
      end
      `mapping_xip_fsm_enumDefinition_defaultEncoding_mapping_xip_fsm_STOP : begin
        if(mapping_xip_fsm_lastFired)begin
          if(mapping_xip_fsm_xipToCtrlCmd_ready)begin
            mapping_xip_fsm_stateNext = `mapping_xip_fsm_enumDefinition_defaultEncoding_mapping_xip_fsm_IDLE;
          end
        end
      end
      default : begin
        mapping_xip_fsm_stateNext = `mapping_xip_fsm_enumDefinition_defaultEncoding_mapping_xip_fsm_IDLE;
      end
    endcase
  end

  assign _zz_26_ = 1'b0;
  assign _zz_28_ = 1'b0;
  always @ (posedge clk_12M) begin
    if(clockCtrl_systemReset) begin
      mapping_interruptCtrl_cmdIntEnable <= 1'b0;
      mapping_interruptCtrl_rspIntEnable <= 1'b0;
      _zz_4_ <= 1'b0;
      _zz_5_ <= 1'b0;
      _zz_6_ <= (1'b0);
      _zz_7_ <= (8'b00000000);
      _zz_8_ <= (8'b00000000);
      _zz_9_ <= (8'b00000000);
      _zz_10_ <= (8'b00000000);
      _zz_11_ <= (1'b0);
      mapping_xip_enable <= 1'b1;
      _zz_14_ <= 1'b0;
      mapping_xip_fsm_xipBusCmdReadyReg <= 1'b0;
      mapping_xip_fsm_counter <= (5'b00000);
      mapping_xip_fsm_stateReg <= `mapping_xip_fsm_enumDefinition_defaultEncoding_boot;
    end else begin
      if(_zz_13_)begin
        _zz_14_ <= (mapping_xip_fsm_xipToCtrlCmd_valid && _zz_12_);
      end
      mapping_xip_fsm_xipBusCmdReadyReg <= 1'b0;
      case(io_apb_PADDR)
        8'b00000000 : begin
        end
        8'b00000100 : begin
        end
        8'b00001100 : begin
          if(_zz_1_)begin
            mapping_interruptCtrl_cmdIntEnable <= _zz_35_[0];
            mapping_interruptCtrl_rspIntEnable <= _zz_36_[0];
          end
        end
        8'b00001000 : begin
          if(_zz_1_)begin
            _zz_4_ <= _zz_37_[0];
            _zz_5_ <= _zz_38_[0];
            _zz_6_ <= io_apb_PWDATA[4 : 4];
          end
        end
        8'b00100000 : begin
          if(_zz_1_)begin
            _zz_7_ <= io_apb_PWDATA[7 : 0];
          end
        end
        8'b00100100 : begin
          if(_zz_1_)begin
            _zz_8_ <= io_apb_PWDATA[7 : 0];
          end
        end
        8'b00101000 : begin
          if(_zz_1_)begin
            _zz_9_ <= io_apb_PWDATA[7 : 0];
          end
        end
        8'b00101100 : begin
          if(_zz_1_)begin
            _zz_10_ <= io_apb_PWDATA[7 : 0];
          end
        end
        8'b00110000 : begin
          if(_zz_1_)begin
            _zz_11_ <= io_apb_PWDATA[0 : 0];
          end
        end
        8'b01000000 : begin
          if(_zz_1_)begin
            mapping_xip_enable <= _zz_39_[0];
          end
        end
        default : begin
        end
      endcase
      mapping_xip_fsm_stateReg <= mapping_xip_fsm_stateNext;
      case(mapping_xip_fsm_stateReg)
        `mapping_xip_fsm_enumDefinition_defaultEncoding_mapping_xip_fsm_IDLE : begin
        end
        `mapping_xip_fsm_enumDefinition_defaultEncoding_mapping_xip_fsm_INSTRUCTION : begin
        end
        `mapping_xip_fsm_enumDefinition_defaultEncoding_mapping_xip_fsm_ADDRESS : begin
          if(mapping_xip_fsm_xipToCtrlCmd_ready)begin
            mapping_xip_fsm_counter <= (mapping_xip_fsm_counter + (5'b00001));
            if(_zz_31_)begin
              mapping_xip_fsm_xipBusCmdReadyReg <= 1'b1;
            end
          end
        end
        `mapping_xip_fsm_enumDefinition_defaultEncoding_mapping_xip_fsm_DUMMY : begin
          if(mapping_xip_fsm_xipToCtrlCmd_ready)begin
            mapping_xip_fsm_counter <= (mapping_xip_fsm_counter + (5'b00001));
          end
        end
        `mapping_xip_fsm_enumDefinition_defaultEncoding_mapping_xip_fsm_PAYLOAD : begin
          if(mapping_xip_fsm_xipToCtrlCmd_ready)begin
            mapping_xip_fsm_counter <= (mapping_xip_fsm_counter + (5'b00001));
          end
        end
        `mapping_xip_fsm_enumDefinition_defaultEncoding_mapping_xip_fsm_STOP : begin
        end
        default : begin
        end
      endcase
      if(((! (mapping_xip_fsm_stateReg == `mapping_xip_fsm_enumDefinition_defaultEncoding_mapping_xip_fsm_ADDRESS)) && (mapping_xip_fsm_stateNext == `mapping_xip_fsm_enumDefinition_defaultEncoding_mapping_xip_fsm_ADDRESS)))begin
        mapping_xip_fsm_counter <= (5'b00000);
      end
      if(((! (mapping_xip_fsm_stateReg == `mapping_xip_fsm_enumDefinition_defaultEncoding_mapping_xip_fsm_DUMMY)) && (mapping_xip_fsm_stateNext == `mapping_xip_fsm_enumDefinition_defaultEncoding_mapping_xip_fsm_DUMMY)))begin
        mapping_xip_fsm_counter <= (5'b00000);
      end
      if(((! (mapping_xip_fsm_stateReg == `mapping_xip_fsm_enumDefinition_defaultEncoding_mapping_xip_fsm_PAYLOAD)) && (mapping_xip_fsm_stateNext == `mapping_xip_fsm_enumDefinition_defaultEncoding_mapping_xip_fsm_PAYLOAD)))begin
        mapping_xip_fsm_counter <= (5'b00000);
      end
    end
  end

  always @ (posedge clk_12M) begin
    if(_zz_13_)begin
      _zz_15_ <= mapping_xip_fsm_xipToCtrlCmd_payload_kind;
      _zz_16_ <= mapping_xip_fsm_xipToCtrlCmd_payload_read;
      _zz_17_ <= mapping_xip_fsm_xipToCtrlCmd_payload_write;
      _zz_18_ <= mapping_xip_fsm_xipToCtrlCmd_payload_data;
    end
    if(mapping_xip_fsm_xipToCtrlCmdBuffer_ready)begin
      mapping_xip_fsm_xipToCtrlModBuffer <= mapping_xip_fsm_xipToCtrlMod;
    end
    if(((mapping_xip_xipBus_rsp_valid && mapping_xip_xipBus_rsp_ready) && mapping_xip_xipBus_rsp_payload_last))begin
      mapping_xip_fsm_lastFired <= 1'b1;
    end
    case(mapping_xip_fsm_stateReg)
      `mapping_xip_fsm_enumDefinition_defaultEncoding_mapping_xip_fsm_IDLE : begin
        mapping_xip_fsm_cmdLength <= mapping_xip_xipBus_cmd_payload_length;
        mapping_xip_fsm_rspCounter <= (5'b00000);
      end
      `mapping_xip_fsm_enumDefinition_defaultEncoding_mapping_xip_fsm_INSTRUCTION : begin
      end
      `mapping_xip_fsm_enumDefinition_defaultEncoding_mapping_xip_fsm_ADDRESS : begin
      end
      `mapping_xip_fsm_enumDefinition_defaultEncoding_mapping_xip_fsm_DUMMY : begin
      end
      `mapping_xip_fsm_enumDefinition_defaultEncoding_mapping_xip_fsm_PAYLOAD : begin
      end
      `mapping_xip_fsm_enumDefinition_defaultEncoding_mapping_xip_fsm_STOP : begin
      end
      default : begin
      end
    endcase
    if(_zz_30_)begin
      if((ctrl_io_rsp_queueWithOccupancy_io_pop_valid && _zz_27_))begin
        mapping_xip_fsm_rspCounter <= (mapping_xip_fsm_rspCounter + (5'b00001));
      end
    end
    if(((! (mapping_xip_fsm_stateReg == `mapping_xip_fsm_enumDefinition_defaultEncoding_mapping_xip_fsm_STOP)) && (mapping_xip_fsm_stateNext == `mapping_xip_fsm_enumDefinition_defaultEncoding_mapping_xip_fsm_STOP)))begin
      mapping_xip_fsm_lastFired <= 1'b0;
    end
  end

endmodule

module BmbArbiter (
      input   io_inputs_0_cmd_valid,
      output  io_inputs_0_cmd_ready,
      input   io_inputs_0_cmd_payload_last,
      input  [0:0] io_inputs_0_cmd_payload_fragment_opcode,
      input  [31:0] io_inputs_0_cmd_payload_fragment_address,
      input  [4:0] io_inputs_0_cmd_payload_fragment_length,
      input  [31:0] io_inputs_0_cmd_payload_fragment_data,
      input  [3:0] io_inputs_0_cmd_payload_fragment_mask,
      input  [0:0] io_inputs_0_cmd_payload_fragment_context,
      output  io_inputs_0_rsp_valid,
      input   io_inputs_0_rsp_ready,
      output  io_inputs_0_rsp_payload_last,
      output [0:0] io_inputs_0_rsp_payload_fragment_opcode,
      output [31:0] io_inputs_0_rsp_payload_fragment_data,
      output [0:0] io_inputs_0_rsp_payload_fragment_context,
      input   io_inputs_1_cmd_valid,
      output  io_inputs_1_cmd_ready,
      input   io_inputs_1_cmd_payload_last,
      input  [0:0] io_inputs_1_cmd_payload_fragment_opcode,
      input  [31:0] io_inputs_1_cmd_payload_fragment_address,
      input  [4:0] io_inputs_1_cmd_payload_fragment_length,
      input  [31:0] io_inputs_1_cmd_payload_fragment_data,
      input  [3:0] io_inputs_1_cmd_payload_fragment_mask,
      input  [0:0] io_inputs_1_cmd_payload_fragment_context,
      output  io_inputs_1_rsp_valid,
      input   io_inputs_1_rsp_ready,
      output  io_inputs_1_rsp_payload_last,
      output [0:0] io_inputs_1_rsp_payload_fragment_opcode,
      output [31:0] io_inputs_1_rsp_payload_fragment_data,
      output [0:0] io_inputs_1_rsp_payload_fragment_context,
      output  io_output_cmd_valid,
      input   io_output_cmd_ready,
      output  io_output_cmd_payload_last,
      output [0:0] io_output_cmd_payload_fragment_source,
      output [0:0] io_output_cmd_payload_fragment_opcode,
      output [31:0] io_output_cmd_payload_fragment_address,
      output [4:0] io_output_cmd_payload_fragment_length,
      output [31:0] io_output_cmd_payload_fragment_data,
      output [3:0] io_output_cmd_payload_fragment_mask,
      output [0:0] io_output_cmd_payload_fragment_context,
      input   io_output_rsp_valid,
      output  io_output_rsp_ready,
      input   io_output_rsp_payload_last,
      input  [0:0] io_output_rsp_payload_fragment_source,
      input  [0:0] io_output_rsp_payload_fragment_opcode,
      input  [31:0] io_output_rsp_payload_fragment_data,
      input  [0:0] io_output_rsp_payload_fragment_context,
      input   clk_12M,
      input   clockCtrl_systemReset);
  wire [0:0] _zz_1_;
  wire [0:0] _zz_2_;
  reg  _zz_3_;
  wire  logic_arbiter_io_inputs_0_ready;
  wire  logic_arbiter_io_inputs_1_ready;
  wire  logic_arbiter_io_output_valid;
  wire  logic_arbiter_io_output_payload_last;
  wire [0:0] logic_arbiter_io_output_payload_fragment_source;
  wire [0:0] logic_arbiter_io_output_payload_fragment_opcode;
  wire [31:0] logic_arbiter_io_output_payload_fragment_address;
  wire [4:0] logic_arbiter_io_output_payload_fragment_length;
  wire [31:0] logic_arbiter_io_output_payload_fragment_data;
  wire [3:0] logic_arbiter_io_output_payload_fragment_mask;
  wire [0:0] logic_arbiter_io_output_payload_fragment_context;
  wire [0:0] logic_arbiter_io_chosen;
  wire [1:0] logic_arbiter_io_chosenOH;
  wire [0:0] logic_rspSel;
  StreamArbiter logic_arbiter ( 
    .io_inputs_0_valid(io_inputs_0_cmd_valid),
    .io_inputs_0_ready(logic_arbiter_io_inputs_0_ready),
    .io_inputs_0_payload_last(io_inputs_0_cmd_payload_last),
    .io_inputs_0_payload_fragment_source(_zz_1_),
    .io_inputs_0_payload_fragment_opcode(io_inputs_0_cmd_payload_fragment_opcode),
    .io_inputs_0_payload_fragment_address(io_inputs_0_cmd_payload_fragment_address),
    .io_inputs_0_payload_fragment_length(io_inputs_0_cmd_payload_fragment_length),
    .io_inputs_0_payload_fragment_data(io_inputs_0_cmd_payload_fragment_data),
    .io_inputs_0_payload_fragment_mask(io_inputs_0_cmd_payload_fragment_mask),
    .io_inputs_0_payload_fragment_context(io_inputs_0_cmd_payload_fragment_context),
    .io_inputs_1_valid(io_inputs_1_cmd_valid),
    .io_inputs_1_ready(logic_arbiter_io_inputs_1_ready),
    .io_inputs_1_payload_last(io_inputs_1_cmd_payload_last),
    .io_inputs_1_payload_fragment_source(_zz_2_),
    .io_inputs_1_payload_fragment_opcode(io_inputs_1_cmd_payload_fragment_opcode),
    .io_inputs_1_payload_fragment_address(io_inputs_1_cmd_payload_fragment_address),
    .io_inputs_1_payload_fragment_length(io_inputs_1_cmd_payload_fragment_length),
    .io_inputs_1_payload_fragment_data(io_inputs_1_cmd_payload_fragment_data),
    .io_inputs_1_payload_fragment_mask(io_inputs_1_cmd_payload_fragment_mask),
    .io_inputs_1_payload_fragment_context(io_inputs_1_cmd_payload_fragment_context),
    .io_output_valid(logic_arbiter_io_output_valid),
    .io_output_ready(io_output_cmd_ready),
    .io_output_payload_last(logic_arbiter_io_output_payload_last),
    .io_output_payload_fragment_source(logic_arbiter_io_output_payload_fragment_source),
    .io_output_payload_fragment_opcode(logic_arbiter_io_output_payload_fragment_opcode),
    .io_output_payload_fragment_address(logic_arbiter_io_output_payload_fragment_address),
    .io_output_payload_fragment_length(logic_arbiter_io_output_payload_fragment_length),
    .io_output_payload_fragment_data(logic_arbiter_io_output_payload_fragment_data),
    .io_output_payload_fragment_mask(logic_arbiter_io_output_payload_fragment_mask),
    .io_output_payload_fragment_context(logic_arbiter_io_output_payload_fragment_context),
    .io_chosen(logic_arbiter_io_chosen),
    .io_chosenOH(logic_arbiter_io_chosenOH),
    .clk_12M(clk_12M),
    .clockCtrl_systemReset(clockCtrl_systemReset) 
  );
  always @(*) begin
    case(logic_rspSel)
      1'b0 : begin
        _zz_3_ = io_inputs_0_rsp_ready;
      end
      default : begin
        _zz_3_ = io_inputs_1_rsp_ready;
      end
    endcase
  end

  assign io_inputs_0_cmd_ready = logic_arbiter_io_inputs_0_ready;
  assign _zz_1_ = (1'b0);
  assign io_inputs_1_cmd_ready = logic_arbiter_io_inputs_1_ready;
  assign _zz_2_ = (1'b0);
  assign io_output_cmd_valid = logic_arbiter_io_output_valid;
  assign io_output_cmd_payload_last = logic_arbiter_io_output_payload_last;
  assign io_output_cmd_payload_fragment_opcode = logic_arbiter_io_output_payload_fragment_opcode;
  assign io_output_cmd_payload_fragment_address = logic_arbiter_io_output_payload_fragment_address;
  assign io_output_cmd_payload_fragment_length = logic_arbiter_io_output_payload_fragment_length;
  assign io_output_cmd_payload_fragment_data = logic_arbiter_io_output_payload_fragment_data;
  assign io_output_cmd_payload_fragment_mask = logic_arbiter_io_output_payload_fragment_mask;
  assign io_output_cmd_payload_fragment_context = logic_arbiter_io_output_payload_fragment_context;
  assign io_output_cmd_payload_fragment_source[0 : 0] = logic_arbiter_io_chosen;
  assign logic_rspSel = io_output_rsp_payload_fragment_source[0 : 0];
  assign io_inputs_0_rsp_valid = (io_output_rsp_valid && (logic_rspSel == (1'b0)));
  assign io_inputs_0_rsp_payload_last = io_output_rsp_payload_last;
  assign io_inputs_0_rsp_payload_fragment_opcode = io_output_rsp_payload_fragment_opcode;
  assign io_inputs_0_rsp_payload_fragment_data = io_output_rsp_payload_fragment_data;
  assign io_inputs_0_rsp_payload_fragment_context = io_output_rsp_payload_fragment_context;
  assign io_inputs_1_rsp_valid = (io_output_rsp_valid && (logic_rspSel == (1'b1)));
  assign io_inputs_1_rsp_payload_last = io_output_rsp_payload_last;
  assign io_inputs_1_rsp_payload_fragment_opcode = io_output_rsp_payload_fragment_opcode;
  assign io_inputs_1_rsp_payload_fragment_data = io_output_rsp_payload_fragment_data;
  assign io_inputs_1_rsp_payload_fragment_context = io_output_rsp_payload_fragment_context;
  assign io_output_rsp_ready = _zz_3_;
endmodule

module BmbIce40Spram (
      input   io_bus_cmd_valid,
      output  io_bus_cmd_ready,
      input   io_bus_cmd_payload_last,
      input  [0:0] io_bus_cmd_payload_fragment_source,
      input  [0:0] io_bus_cmd_payload_fragment_opcode,
      input  [16:0] io_bus_cmd_payload_fragment_address,
      input  [1:0] io_bus_cmd_payload_fragment_length,
      input  [31:0] io_bus_cmd_payload_fragment_data,
      input  [3:0] io_bus_cmd_payload_fragment_mask,
      input  [2:0] io_bus_cmd_payload_fragment_context,
      output  io_bus_rsp_valid,
      input   io_bus_rsp_ready,
      output  io_bus_rsp_payload_last,
      output [0:0] io_bus_rsp_payload_fragment_source,
      output [0:0] io_bus_rsp_payload_fragment_opcode,
      output [31:0] io_bus_rsp_payload_fragment_data,
      output [2:0] io_bus_rsp_payload_fragment_context,
      input   clk_12M,
      input   clockCtrl_systemReset);
  wire [15:0] _zz_1_;
  wire [13:0] _zz_2_;
  wire [3:0] _zz_3_;
  wire  _zz_4_;
  wire  _zz_5_;
  wire  _zz_6_;
  wire  _zz_7_;
  wire  _zz_8_;
  wire [15:0] _zz_9_;
  wire [13:0] _zz_10_;
  wire [3:0] _zz_11_;
  wire  _zz_12_;
  wire  _zz_13_;
  wire  _zz_14_;
  wire  _zz_15_;
  wire  _zz_16_;
  wire [15:0] _zz_17_;
  wire [13:0] _zz_18_;
  wire [3:0] _zz_19_;
  wire  _zz_20_;
  wire  _zz_21_;
  wire  _zz_22_;
  wire  _zz_23_;
  wire  _zz_24_;
  wire [15:0] _zz_25_;
  wire [13:0] _zz_26_;
  wire [3:0] _zz_27_;
  wire  _zz_28_;
  wire  _zz_29_;
  wire  _zz_30_;
  wire  _zz_31_;
  wire  _zz_32_;
  reg [31:0] _zz_33_;
  wire [15:0] banks_0_mems_0_DATAOUT;
  wire [15:0] banks_0_mems_1_DATAOUT;
  wire [15:0] banks_1_mems_0_DATAOUT;
  wire [15:0] banks_1_mems_1_DATAOUT;
  wire [14:0] _zz_34_;
  wire [14:0] _zz_35_;
  wire [14:0] _zz_36_;
  wire [14:0] _zz_37_;
  wire [0:0] bankSel;
  wire [31:0] banks_0_readData;
  wire [31:0] banks_1_readData;
  reg [0:0] rspBankSel;
  reg  io_bus_cmd_valid_regNextWhen;
  reg [0:0] io_bus_cmd_payload_fragment_source_regNextWhen;
  reg [2:0] io_bus_cmd_payload_fragment_context_regNextWhen;
  assign _zz_34_ = (io_bus_cmd_payload_fragment_address >>> 2);
  assign _zz_35_ = (io_bus_cmd_payload_fragment_address >>> 2);
  assign _zz_36_ = (io_bus_cmd_payload_fragment_address >>> 2);
  assign _zz_37_ = (io_bus_cmd_payload_fragment_address >>> 2);
  SB_SPRAM256KA banks_0_mems_0 ( 
    .DATAIN(_zz_1_),
    .ADDRESS(_zz_2_),
    .MASKWREN(_zz_3_),
    .WREN(_zz_4_),
    .CHIPSELECT(_zz_5_),
    .CLOCK(clk_12M),
    .DATAOUT(banks_0_mems_0_DATAOUT),
    .STANDBY(_zz_6_),
    .SLEEP(_zz_7_),
    .POWEROFF(_zz_8_) 
  );
  SB_SPRAM256KA banks_0_mems_1 ( 
    .DATAIN(_zz_9_),
    .ADDRESS(_zz_10_),
    .MASKWREN(_zz_11_),
    .WREN(_zz_12_),
    .CHIPSELECT(_zz_13_),
    .CLOCK(clk_12M),
    .DATAOUT(banks_0_mems_1_DATAOUT),
    .STANDBY(_zz_14_),
    .SLEEP(_zz_15_),
    .POWEROFF(_zz_16_) 
  );
  SB_SPRAM256KA banks_1_mems_0 ( 
    .DATAIN(_zz_17_),
    .ADDRESS(_zz_18_),
    .MASKWREN(_zz_19_),
    .WREN(_zz_20_),
    .CHIPSELECT(_zz_21_),
    .CLOCK(clk_12M),
    .DATAOUT(banks_1_mems_0_DATAOUT),
    .STANDBY(_zz_22_),
    .SLEEP(_zz_23_),
    .POWEROFF(_zz_24_) 
  );
  SB_SPRAM256KA banks_1_mems_1 ( 
    .DATAIN(_zz_25_),
    .ADDRESS(_zz_26_),
    .MASKWREN(_zz_27_),
    .WREN(_zz_28_),
    .CHIPSELECT(_zz_29_),
    .CLOCK(clk_12M),
    .DATAOUT(banks_1_mems_1_DATAOUT),
    .STANDBY(_zz_30_),
    .SLEEP(_zz_31_),
    .POWEROFF(_zz_32_) 
  );
  always @(*) begin
    case(rspBankSel)
      1'b0 : begin
        _zz_33_ = banks_0_readData;
      end
      default : begin
        _zz_33_ = banks_1_readData;
      end
    endcase
  end

  assign bankSel = (io_bus_cmd_payload_fragment_address >>> 16);
  assign _zz_1_ = io_bus_cmd_payload_fragment_data[15 : 0];
  assign _zz_3_ = {{{io_bus_cmd_payload_fragment_mask[1],io_bus_cmd_payload_fragment_mask[1]},io_bus_cmd_payload_fragment_mask[0]},io_bus_cmd_payload_fragment_mask[0]};
  assign _zz_9_ = io_bus_cmd_payload_fragment_data[31 : 16];
  assign _zz_11_ = {{{io_bus_cmd_payload_fragment_mask[3],io_bus_cmd_payload_fragment_mask[3]},io_bus_cmd_payload_fragment_mask[2]},io_bus_cmd_payload_fragment_mask[2]};
  assign _zz_5_ = (io_bus_cmd_valid && (bankSel == (1'b0)));
  assign _zz_2_ = _zz_34_[13:0];
  assign _zz_4_ = (io_bus_cmd_payload_fragment_opcode == (1'b1));
  assign _zz_6_ = 1'b0;
  assign _zz_7_ = 1'b0;
  assign _zz_8_ = 1'b1;
  assign _zz_13_ = (io_bus_cmd_valid && (bankSel == (1'b0)));
  assign _zz_10_ = _zz_35_[13:0];
  assign _zz_12_ = (io_bus_cmd_payload_fragment_opcode == (1'b1));
  assign _zz_14_ = 1'b0;
  assign _zz_15_ = 1'b0;
  assign _zz_16_ = 1'b1;
  assign banks_0_readData = {banks_0_mems_1_DATAOUT,banks_0_mems_0_DATAOUT};
  assign _zz_17_ = io_bus_cmd_payload_fragment_data[15 : 0];
  assign _zz_19_ = {{{io_bus_cmd_payload_fragment_mask[1],io_bus_cmd_payload_fragment_mask[1]},io_bus_cmd_payload_fragment_mask[0]},io_bus_cmd_payload_fragment_mask[0]};
  assign _zz_25_ = io_bus_cmd_payload_fragment_data[31 : 16];
  assign _zz_27_ = {{{io_bus_cmd_payload_fragment_mask[3],io_bus_cmd_payload_fragment_mask[3]},io_bus_cmd_payload_fragment_mask[2]},io_bus_cmd_payload_fragment_mask[2]};
  assign _zz_21_ = (io_bus_cmd_valid && (bankSel == (1'b1)));
  assign _zz_18_ = _zz_36_[13:0];
  assign _zz_20_ = (io_bus_cmd_payload_fragment_opcode == (1'b1));
  assign _zz_22_ = 1'b0;
  assign _zz_23_ = 1'b0;
  assign _zz_24_ = 1'b1;
  assign _zz_29_ = (io_bus_cmd_valid && (bankSel == (1'b1)));
  assign _zz_26_ = _zz_37_[13:0];
  assign _zz_28_ = (io_bus_cmd_payload_fragment_opcode == (1'b1));
  assign _zz_30_ = 1'b0;
  assign _zz_31_ = 1'b0;
  assign _zz_32_ = 1'b1;
  assign banks_1_readData = {banks_1_mems_1_DATAOUT,banks_1_mems_0_DATAOUT};
  assign io_bus_cmd_ready = (! (io_bus_rsp_valid && (! io_bus_rsp_ready)));
  assign io_bus_rsp_valid = io_bus_cmd_valid_regNextWhen;
  assign io_bus_rsp_payload_fragment_source = io_bus_cmd_payload_fragment_source_regNextWhen;
  assign io_bus_rsp_payload_fragment_context = io_bus_cmd_payload_fragment_context_regNextWhen;
  assign io_bus_rsp_payload_fragment_data = _zz_33_;
  assign io_bus_rsp_payload_fragment_opcode = (1'b0);
  assign io_bus_rsp_payload_last = 1'b1;
  always @ (posedge clk_12M) begin
    if(io_bus_cmd_ready)begin
      rspBankSel <= (io_bus_cmd_payload_fragment_address >>> 16);
    end
    if(io_bus_cmd_ready)begin
      io_bus_cmd_payload_fragment_source_regNextWhen <= io_bus_cmd_payload_fragment_source;
    end
    if(io_bus_cmd_ready)begin
      io_bus_cmd_payload_fragment_context_regNextWhen <= io_bus_cmd_payload_fragment_context;
    end
  end

  always @ (posedge clk_12M) begin
    if(clockCtrl_systemReset) begin
      io_bus_cmd_valid_regNextWhen <= 1'b0;
    end else begin
      if(io_bus_cmd_ready)begin
        io_bus_cmd_valid_regNextWhen <= io_bus_cmd_valid;
      end
    end
  end

endmodule

module BmbArbiter_1_ (
      input   io_inputs_0_cmd_valid,
      output  io_inputs_0_cmd_ready,
      input   io_inputs_0_cmd_payload_last,
      input  [0:0] io_inputs_0_cmd_payload_fragment_source,
      input  [0:0] io_inputs_0_cmd_payload_fragment_opcode,
      input  [16:0] io_inputs_0_cmd_payload_fragment_address,
      input  [4:0] io_inputs_0_cmd_payload_fragment_length,
      input  [31:0] io_inputs_0_cmd_payload_fragment_data,
      input  [3:0] io_inputs_0_cmd_payload_fragment_mask,
      input  [0:0] io_inputs_0_cmd_payload_fragment_context,
      output  io_inputs_0_rsp_valid,
      input   io_inputs_0_rsp_ready,
      output  io_inputs_0_rsp_payload_last,
      output [0:0] io_inputs_0_rsp_payload_fragment_source,
      output [0:0] io_inputs_0_rsp_payload_fragment_opcode,
      output [31:0] io_inputs_0_rsp_payload_fragment_data,
      output [0:0] io_inputs_0_rsp_payload_fragment_context,
      output  io_output_cmd_valid,
      input   io_output_cmd_ready,
      output  io_output_cmd_payload_last,
      output [0:0] io_output_cmd_payload_fragment_source,
      output [0:0] io_output_cmd_payload_fragment_opcode,
      output [16:0] io_output_cmd_payload_fragment_address,
      output [4:0] io_output_cmd_payload_fragment_length,
      output [31:0] io_output_cmd_payload_fragment_data,
      output [3:0] io_output_cmd_payload_fragment_mask,
      output [0:0] io_output_cmd_payload_fragment_context,
      input   io_output_rsp_valid,
      output  io_output_rsp_ready,
      input   io_output_rsp_payload_last,
      input  [0:0] io_output_rsp_payload_fragment_source,
      input  [0:0] io_output_rsp_payload_fragment_opcode,
      input  [31:0] io_output_rsp_payload_fragment_data,
      input  [0:0] io_output_rsp_payload_fragment_context);
  assign io_output_cmd_valid = io_inputs_0_cmd_valid;
  assign io_inputs_0_cmd_ready = io_output_cmd_ready;
  assign io_inputs_0_rsp_valid = io_output_rsp_valid;
  assign io_output_rsp_ready = io_inputs_0_rsp_ready;
  assign io_output_cmd_payload_last = io_inputs_0_cmd_payload_last;
  assign io_inputs_0_rsp_payload_last = io_output_rsp_payload_last;
  assign io_output_cmd_payload_fragment_source = io_inputs_0_cmd_payload_fragment_source;
  assign io_output_cmd_payload_fragment_opcode = io_inputs_0_cmd_payload_fragment_opcode;
  assign io_output_cmd_payload_fragment_address = io_inputs_0_cmd_payload_fragment_address;
  assign io_output_cmd_payload_fragment_length = io_inputs_0_cmd_payload_fragment_length;
  assign io_output_cmd_payload_fragment_data = io_inputs_0_cmd_payload_fragment_data;
  assign io_output_cmd_payload_fragment_mask = io_inputs_0_cmd_payload_fragment_mask;
  assign io_output_cmd_payload_fragment_context = io_inputs_0_cmd_payload_fragment_context;
  assign io_inputs_0_rsp_payload_fragment_source = io_output_rsp_payload_fragment_source;
  assign io_inputs_0_rsp_payload_fragment_opcode = io_output_rsp_payload_fragment_opcode;
  assign io_inputs_0_rsp_payload_fragment_data = io_output_rsp_payload_fragment_data;
  assign io_inputs_0_rsp_payload_fragment_context = io_output_rsp_payload_fragment_context;
endmodule

module BmbUnburstify (
      input   io_input_cmd_valid,
      output reg  io_input_cmd_ready,
      input   io_input_cmd_payload_last,
      input  [0:0] io_input_cmd_payload_fragment_source,
      input  [0:0] io_input_cmd_payload_fragment_opcode,
      input  [16:0] io_input_cmd_payload_fragment_address,
      input  [4:0] io_input_cmd_payload_fragment_length,
      input  [31:0] io_input_cmd_payload_fragment_data,
      input  [3:0] io_input_cmd_payload_fragment_mask,
      input  [0:0] io_input_cmd_payload_fragment_context,
      output  io_input_rsp_valid,
      input   io_input_rsp_ready,
      output  io_input_rsp_payload_last,
      output [0:0] io_input_rsp_payload_fragment_source,
      output [0:0] io_input_rsp_payload_fragment_opcode,
      output [31:0] io_input_rsp_payload_fragment_data,
      output [0:0] io_input_rsp_payload_fragment_context,
      output reg  io_output_cmd_valid,
      input   io_output_cmd_ready,
      output  io_output_cmd_payload_last,
      output reg [0:0] io_output_cmd_payload_fragment_source,
      output reg [0:0] io_output_cmd_payload_fragment_opcode,
      output reg [16:0] io_output_cmd_payload_fragment_address,
      output reg [1:0] io_output_cmd_payload_fragment_length,
      output [31:0] io_output_cmd_payload_fragment_data,
      output [3:0] io_output_cmd_payload_fragment_mask,
      output reg [2:0] io_output_cmd_payload_fragment_context,
      input   io_output_rsp_valid,
      output  io_output_rsp_ready,
      input   io_output_rsp_payload_last,
      input  [0:0] io_output_rsp_payload_fragment_source,
      input  [0:0] io_output_rsp_payload_fragment_opcode,
      input  [31:0] io_output_rsp_payload_fragment_data,
      input  [2:0] io_output_rsp_payload_fragment_context,
      input   clk_12M,
      input   clockCtrl_systemReset);
  wire  _zz_1_;
  wire [11:0] _zz_2_;
  wire [11:0] _zz_3_;
  wire [11:0] _zz_4_;
  wire  doResult;
  reg  buffer_valid;
  reg [0:0] buffer_opcode;
  reg [0:0] buffer_source;
  reg [16:0] buffer_address;
  reg [0:0] buffer_context;
  reg [2:0] buffer_beat;
  wire  buffer_last;
  wire [16:0] buffer_addressIncr;
  wire  buffer_isWrite;
  wire [2:0] cmdTransferBeatCount;
  wire  requireBuffer;
  assign _zz_1_ = (io_output_cmd_valid && io_output_cmd_ready);
  assign _zz_2_ = (_zz_4_ + (12'b000000000100));
  assign _zz_3_ = buffer_address[11 : 0];
  assign _zz_4_ = _zz_3_;
  assign buffer_last = (buffer_beat == (3'b001));
  assign buffer_addressIncr = {buffer_address[16 : 12],(_zz_2_ & (~ (12'b000000000011)))};
  assign buffer_isWrite = (buffer_opcode == (1'b1));
  assign cmdTransferBeatCount = io_input_cmd_payload_fragment_length[4 : 2];
  assign requireBuffer = (cmdTransferBeatCount != (3'b000));
  assign io_output_cmd_payload_fragment_data = io_input_cmd_payload_fragment_data;
  assign io_output_cmd_payload_fragment_mask = io_input_cmd_payload_fragment_mask;
  assign io_output_cmd_payload_last = 1'b1;
  always @ (*) begin
    if(buffer_valid)begin
      io_output_cmd_payload_fragment_source = buffer_source;
    end else begin
      io_output_cmd_payload_fragment_source = io_input_cmd_payload_fragment_source;
    end
  end

  always @ (*) begin
    if(buffer_valid)begin
      io_output_cmd_payload_fragment_address = buffer_addressIncr;
    end else begin
      io_output_cmd_payload_fragment_address = io_input_cmd_payload_fragment_address;
      if(requireBuffer)begin
        io_output_cmd_payload_fragment_address[1 : 0] = (2'b00);
      end
    end
  end

  always @ (*) begin
    if(buffer_valid)begin
      io_output_cmd_payload_fragment_opcode = buffer_opcode;
    end else begin
      io_output_cmd_payload_fragment_opcode = io_input_cmd_payload_fragment_opcode;
    end
  end

  always @ (*) begin
    if(buffer_valid)begin
      io_output_cmd_payload_fragment_length = (2'b11);
    end else begin
      if(requireBuffer)begin
        io_output_cmd_payload_fragment_length = (2'b11);
      end else begin
        io_output_cmd_payload_fragment_length = io_input_cmd_payload_fragment_length[1:0];
      end
    end
  end

  always @ (*) begin
    if(buffer_valid)begin
      io_output_cmd_payload_fragment_context[0 : 0] = buffer_context;
    end else begin
      io_output_cmd_payload_fragment_context[0 : 0] = io_input_cmd_payload_fragment_context;
    end
    if(buffer_valid)begin
      io_output_cmd_payload_fragment_context[2] = buffer_last;
      io_output_cmd_payload_fragment_context[1] = buffer_isWrite;
    end else begin
      io_output_cmd_payload_fragment_context[1] = (io_input_cmd_payload_fragment_opcode == (1'b1));
      io_output_cmd_payload_fragment_context[2] = (! requireBuffer);
    end
  end

  always @ (*) begin
    io_input_cmd_ready = 1'b0;
    if(buffer_valid)begin
      io_input_cmd_ready = (buffer_isWrite && io_output_cmd_ready);
    end else begin
      io_input_cmd_ready = io_output_cmd_ready;
    end
  end

  always @ (*) begin
    if(buffer_valid)begin
      io_output_cmd_valid = (! (buffer_isWrite && (! io_input_cmd_valid)));
    end else begin
      io_output_cmd_valid = io_input_cmd_valid;
    end
  end

  assign io_input_rsp_valid = (io_output_rsp_valid && (io_output_rsp_payload_fragment_context[2] || (! io_output_rsp_payload_fragment_context[1])));
  assign io_input_rsp_payload_last = io_output_rsp_payload_fragment_context[2];
  assign io_input_rsp_payload_fragment_source = io_output_rsp_payload_fragment_source;
  assign io_input_rsp_payload_fragment_opcode = io_output_rsp_payload_fragment_opcode;
  assign io_input_rsp_payload_fragment_data = io_output_rsp_payload_fragment_data;
  assign io_input_rsp_payload_fragment_context = io_output_rsp_payload_fragment_context[0:0];
  assign io_output_rsp_ready = io_input_rsp_ready;
  always @ (posedge clk_12M) begin
    if(clockCtrl_systemReset) begin
      buffer_valid <= 1'b0;
    end else begin
      if(_zz_1_)begin
        if(buffer_last)begin
          buffer_valid <= 1'b0;
        end
      end
      if(! buffer_valid) begin
        buffer_valid <= (requireBuffer && (io_output_cmd_valid && io_output_cmd_ready));
      end
    end
  end

  always @ (posedge clk_12M) begin
    if(_zz_1_)begin
      buffer_beat <= (buffer_beat - (3'b001));
      buffer_address[11 : 0] <= buffer_addressIncr[11 : 0];
    end
    if(! buffer_valid) begin
      buffer_opcode <= io_input_cmd_payload_fragment_opcode;
      buffer_source <= io_input_cmd_payload_fragment_source;
      buffer_address <= io_input_cmd_payload_fragment_address;
      buffer_context <= io_input_cmd_payload_fragment_context;
      buffer_beat <= cmdTransferBeatCount;
    end
  end

endmodule

module BmbDecoder_2_ (
      input   io_input_cmd_valid,
      output reg  io_input_cmd_ready,
      input   io_input_cmd_payload_last,
      input  [0:0] io_input_cmd_payload_fragment_source,
      input  [0:0] io_input_cmd_payload_fragment_opcode,
      input  [31:0] io_input_cmd_payload_fragment_address,
      input  [4:0] io_input_cmd_payload_fragment_length,
      input  [31:0] io_input_cmd_payload_fragment_data,
      input  [3:0] io_input_cmd_payload_fragment_mask,
      input  [0:0] io_input_cmd_payload_fragment_context,
      output reg  io_input_rsp_valid,
      input   io_input_rsp_ready,
      output reg  io_input_rsp_payload_last,
      output reg [0:0] io_input_rsp_payload_fragment_source,
      output reg [0:0] io_input_rsp_payload_fragment_opcode,
      output [31:0] io_input_rsp_payload_fragment_data,
      output reg [0:0] io_input_rsp_payload_fragment_context,
      output reg  io_outputs_0_cmd_valid,
      input   io_outputs_0_cmd_ready,
      output  io_outputs_0_cmd_payload_last,
      output [0:0] io_outputs_0_cmd_payload_fragment_source,
      output [0:0] io_outputs_0_cmd_payload_fragment_opcode,
      output [31:0] io_outputs_0_cmd_payload_fragment_address,
      output [4:0] io_outputs_0_cmd_payload_fragment_length,
      output [31:0] io_outputs_0_cmd_payload_fragment_data,
      output [3:0] io_outputs_0_cmd_payload_fragment_mask,
      output [0:0] io_outputs_0_cmd_payload_fragment_context,
      input   io_outputs_0_rsp_valid,
      output  io_outputs_0_rsp_ready,
      input   io_outputs_0_rsp_payload_last,
      input  [0:0] io_outputs_0_rsp_payload_fragment_source,
      input  [0:0] io_outputs_0_rsp_payload_fragment_opcode,
      input  [31:0] io_outputs_0_rsp_payload_fragment_data,
      input  [0:0] io_outputs_0_rsp_payload_fragment_context,
      output reg  io_outputs_1_cmd_valid,
      input   io_outputs_1_cmd_ready,
      output  io_outputs_1_cmd_payload_last,
      output [0:0] io_outputs_1_cmd_payload_fragment_source,
      output [0:0] io_outputs_1_cmd_payload_fragment_opcode,
      output [31:0] io_outputs_1_cmd_payload_fragment_address,
      output [4:0] io_outputs_1_cmd_payload_fragment_length,
      output [31:0] io_outputs_1_cmd_payload_fragment_data,
      output [3:0] io_outputs_1_cmd_payload_fragment_mask,
      output [0:0] io_outputs_1_cmd_payload_fragment_context,
      input   io_outputs_1_rsp_valid,
      output  io_outputs_1_rsp_ready,
      input   io_outputs_1_rsp_payload_last,
      input  [0:0] io_outputs_1_rsp_payload_fragment_source,
      input  [0:0] io_outputs_1_rsp_payload_fragment_opcode,
      input  [31:0] io_outputs_1_rsp_payload_fragment_data,
      input  [0:0] io_outputs_1_rsp_payload_fragment_context,
      output reg  io_outputs_2_cmd_valid,
      input   io_outputs_2_cmd_ready,
      output  io_outputs_2_cmd_payload_last,
      output [0:0] io_outputs_2_cmd_payload_fragment_source,
      output [0:0] io_outputs_2_cmd_payload_fragment_opcode,
      output [31:0] io_outputs_2_cmd_payload_fragment_address,
      output [4:0] io_outputs_2_cmd_payload_fragment_length,
      output [31:0] io_outputs_2_cmd_payload_fragment_data,
      output [3:0] io_outputs_2_cmd_payload_fragment_mask,
      output [0:0] io_outputs_2_cmd_payload_fragment_context,
      input   io_outputs_2_rsp_valid,
      output  io_outputs_2_rsp_ready,
      input   io_outputs_2_rsp_payload_last,
      input  [0:0] io_outputs_2_rsp_payload_fragment_source,
      input  [0:0] io_outputs_2_rsp_payload_fragment_opcode,
      input  [31:0] io_outputs_2_rsp_payload_fragment_data,
      input  [0:0] io_outputs_2_rsp_payload_fragment_context,
      input   clk_12M,
      input   clockCtrl_systemReset);
  reg  _zz_5_;
  reg [0:0] _zz_6_;
  reg [0:0] _zz_7_;
  reg [31:0] _zz_8_;
  reg [0:0] _zz_9_;
  wire [3:0] _zz_10_;
  wire [0:0] _zz_11_;
  wire [3:0] _zz_12_;
  wire [0:0] _zz_13_;
  wire [3:0] _zz_14_;
  wire  logic_hits_0;
  wire  logic_hits_1;
  reg  logic_hits_2;
  wire  _zz_1_;
  wire  _zz_2_;
  wire  _zz_3_;
  wire  logic_noHit;
  reg [3:0] logic_rspPendingCounter;
  reg  logic_rspHits_0;
  reg  logic_rspHits_1;
  reg  logic_rspHits_2;
  wire  logic_rspPending;
  wire  logic_rspNoHitValid;
  reg  logic_rspNoHit_doIt;
  reg  logic_rspNoHit_singleBeatRsp;
  reg [0:0] logic_rspNoHit_source;
  reg [0:0] logic_rspNoHit_context;
  reg [2:0] logic_rspNoHit_counter;
  wire [1:0] _zz_4_;
  wire  logic_cmdWait;
  assign _zz_10_ = (logic_rspPendingCounter + _zz_12_);
  assign _zz_11_ = ((io_input_cmd_valid && io_input_cmd_ready) && io_input_cmd_payload_last);
  assign _zz_12_ = {3'd0, _zz_11_};
  assign _zz_13_ = ((io_input_rsp_valid && io_input_rsp_ready) && io_input_rsp_payload_last);
  assign _zz_14_ = {3'd0, _zz_13_};
  always @(*) begin
    case(_zz_4_)
      2'b00 : begin
        _zz_5_ = io_outputs_0_rsp_payload_last;
        _zz_6_ = io_outputs_0_rsp_payload_fragment_source;
        _zz_7_ = io_outputs_0_rsp_payload_fragment_opcode;
        _zz_8_ = io_outputs_0_rsp_payload_fragment_data;
        _zz_9_ = io_outputs_0_rsp_payload_fragment_context;
      end
      2'b01 : begin
        _zz_5_ = io_outputs_1_rsp_payload_last;
        _zz_6_ = io_outputs_1_rsp_payload_fragment_source;
        _zz_7_ = io_outputs_1_rsp_payload_fragment_opcode;
        _zz_8_ = io_outputs_1_rsp_payload_fragment_data;
        _zz_9_ = io_outputs_1_rsp_payload_fragment_context;
      end
      default : begin
        _zz_5_ = io_outputs_2_rsp_payload_last;
        _zz_6_ = io_outputs_2_rsp_payload_fragment_source;
        _zz_7_ = io_outputs_2_rsp_payload_fragment_opcode;
        _zz_8_ = io_outputs_2_rsp_payload_fragment_data;
        _zz_9_ = io_outputs_2_rsp_payload_fragment_context;
      end
    endcase
  end

  assign logic_hits_0 = ((io_input_cmd_payload_fragment_address & (~ (32'b00000000000000011111111111111111))) == (32'b10000000000000000000000000000000));
  always @ (*) begin
    io_outputs_0_cmd_valid = (io_input_cmd_valid && logic_hits_0);
    if(logic_cmdWait)begin
      io_outputs_0_cmd_valid = 1'b0;
    end
  end

  assign _zz_1_ = io_input_cmd_payload_last;
  assign io_outputs_0_cmd_payload_last = _zz_1_;
  assign io_outputs_0_cmd_payload_fragment_source = io_input_cmd_payload_fragment_source;
  assign io_outputs_0_cmd_payload_fragment_opcode = io_input_cmd_payload_fragment_opcode;
  assign io_outputs_0_cmd_payload_fragment_address = io_input_cmd_payload_fragment_address;
  assign io_outputs_0_cmd_payload_fragment_length = io_input_cmd_payload_fragment_length;
  assign io_outputs_0_cmd_payload_fragment_data = io_input_cmd_payload_fragment_data;
  assign io_outputs_0_cmd_payload_fragment_mask = io_input_cmd_payload_fragment_mask;
  assign io_outputs_0_cmd_payload_fragment_context = io_input_cmd_payload_fragment_context;
  assign logic_hits_1 = ((io_input_cmd_payload_fragment_address & (~ (32'b00000000111111111111111111111111))) == (32'b00010000000000000000000000000000));
  always @ (*) begin
    io_outputs_1_cmd_valid = (io_input_cmd_valid && logic_hits_1);
    if(logic_cmdWait)begin
      io_outputs_1_cmd_valid = 1'b0;
    end
  end

  assign _zz_2_ = io_input_cmd_payload_last;
  assign io_outputs_1_cmd_payload_last = _zz_2_;
  assign io_outputs_1_cmd_payload_fragment_source = io_input_cmd_payload_fragment_source;
  assign io_outputs_1_cmd_payload_fragment_opcode = io_input_cmd_payload_fragment_opcode;
  assign io_outputs_1_cmd_payload_fragment_address = io_input_cmd_payload_fragment_address;
  assign io_outputs_1_cmd_payload_fragment_length = io_input_cmd_payload_fragment_length;
  assign io_outputs_1_cmd_payload_fragment_data = io_input_cmd_payload_fragment_data;
  assign io_outputs_1_cmd_payload_fragment_mask = io_input_cmd_payload_fragment_mask;
  assign io_outputs_1_cmd_payload_fragment_context = io_input_cmd_payload_fragment_context;
  always @ (*) begin
    logic_hits_2 = ((io_input_cmd_payload_fragment_address & (~ (32'b00000000111111111111111111111111))) == (32'b00100000000000000000000000000000));
    if((io_input_cmd_payload_fragment_opcode == (1'b1)))begin
      logic_hits_2 = 1'b0;
    end
  end

  always @ (*) begin
    io_outputs_2_cmd_valid = (io_input_cmd_valid && logic_hits_2);
    if(logic_cmdWait)begin
      io_outputs_2_cmd_valid = 1'b0;
    end
  end

  assign _zz_3_ = io_input_cmd_payload_last;
  assign io_outputs_2_cmd_payload_last = _zz_3_;
  assign io_outputs_2_cmd_payload_fragment_source = io_input_cmd_payload_fragment_source;
  assign io_outputs_2_cmd_payload_fragment_opcode = io_input_cmd_payload_fragment_opcode;
  assign io_outputs_2_cmd_payload_fragment_address = io_input_cmd_payload_fragment_address;
  assign io_outputs_2_cmd_payload_fragment_length = io_input_cmd_payload_fragment_length;
  assign io_outputs_2_cmd_payload_fragment_data = io_input_cmd_payload_fragment_data;
  assign io_outputs_2_cmd_payload_fragment_mask = io_input_cmd_payload_fragment_mask;
  assign io_outputs_2_cmd_payload_fragment_context = io_input_cmd_payload_fragment_context;
  assign logic_noHit = (! ({logic_hits_2,{logic_hits_1,logic_hits_0}} != (3'b000)));
  always @ (*) begin
    io_input_cmd_ready = (({(logic_hits_2 && io_outputs_2_cmd_ready),{(logic_hits_1 && io_outputs_1_cmd_ready),(logic_hits_0 && io_outputs_0_cmd_ready)}} != (3'b000)) || logic_noHit);
    if(logic_cmdWait)begin
      io_input_cmd_ready = 1'b0;
    end
  end

  assign logic_rspPending = (logic_rspPendingCounter != (4'b0000));
  assign logic_rspNoHitValid = (! ({logic_rspHits_2,{logic_rspHits_1,logic_rspHits_0}} != (3'b000)));
  always @ (*) begin
    io_input_rsp_valid = (({io_outputs_2_rsp_valid,{io_outputs_1_rsp_valid,io_outputs_0_rsp_valid}} != (3'b000)) || (logic_rspPending && logic_rspNoHitValid));
    if(logic_rspNoHit_doIt)begin
      io_input_rsp_valid = 1'b1;
    end
  end

  assign _zz_4_ = {logic_rspHits_2,logic_rspHits_1};
  always @ (*) begin
    io_input_rsp_payload_last = _zz_5_;
    if(logic_rspNoHit_doIt)begin
      io_input_rsp_payload_last = 1'b0;
      if((logic_rspNoHit_counter == (3'b000)))begin
        io_input_rsp_payload_last = 1'b1;
      end
      if(logic_rspNoHit_singleBeatRsp)begin
        io_input_rsp_payload_last = 1'b1;
      end
    end
  end

  always @ (*) begin
    io_input_rsp_payload_fragment_source = _zz_6_;
    if(logic_rspNoHit_doIt)begin
      io_input_rsp_payload_fragment_source = logic_rspNoHit_source;
    end
  end

  always @ (*) begin
    io_input_rsp_payload_fragment_opcode = _zz_7_;
    if(logic_rspNoHit_doIt)begin
      io_input_rsp_payload_fragment_opcode = (1'b1);
    end
  end

  assign io_input_rsp_payload_fragment_data = _zz_8_;
  always @ (*) begin
    io_input_rsp_payload_fragment_context = _zz_9_;
    if(logic_rspNoHit_doIt)begin
      io_input_rsp_payload_fragment_context = logic_rspNoHit_context;
    end
  end

  assign io_outputs_0_rsp_ready = io_input_rsp_ready;
  assign io_outputs_1_rsp_ready = io_input_rsp_ready;
  assign io_outputs_2_rsp_ready = io_input_rsp_ready;
  assign logic_cmdWait = ((logic_rspPending && ((((logic_hits_0 != logic_rspHits_0) || (logic_hits_1 != logic_rspHits_1)) || (logic_hits_2 != logic_rspHits_2)) || logic_rspNoHitValid)) || (logic_rspPendingCounter == (4'b1111)));
  always @ (posedge clk_12M) begin
    if(clockCtrl_systemReset) begin
      logic_rspPendingCounter <= (4'b0000);
      logic_rspNoHit_doIt <= 1'b0;
    end else begin
      logic_rspPendingCounter <= (_zz_10_ - _zz_14_);
      if(((io_input_rsp_valid && io_input_rsp_ready) && io_input_rsp_payload_last))begin
        logic_rspNoHit_doIt <= 1'b0;
      end
      if((((io_input_cmd_valid && io_input_cmd_ready) && logic_noHit) && io_input_cmd_payload_last))begin
        logic_rspNoHit_doIt <= 1'b1;
      end
    end
  end

  always @ (posedge clk_12M) begin
    if((io_input_cmd_valid && io_input_cmd_ready))begin
      logic_rspHits_0 <= logic_hits_0;
      logic_rspHits_1 <= logic_hits_1;
      logic_rspHits_2 <= logic_hits_2;
    end
    if((io_input_cmd_valid && io_input_cmd_ready))begin
      logic_rspNoHit_singleBeatRsp <= (io_input_cmd_payload_fragment_opcode == (1'b1));
    end
    if((io_input_cmd_valid && io_input_cmd_ready))begin
      logic_rspNoHit_source <= io_input_cmd_payload_fragment_source;
    end
    if((io_input_cmd_valid && io_input_cmd_ready))begin
      logic_rspNoHit_context <= io_input_cmd_payload_fragment_context;
    end
    if((io_input_cmd_valid && io_input_cmd_ready))begin
      logic_rspNoHit_counter <= io_input_cmd_payload_fragment_length[4 : 2];
    end
    if(logic_rspNoHit_doIt)begin
      if((io_input_rsp_valid && io_input_rsp_ready))begin
        logic_rspNoHit_counter <= (logic_rspNoHit_counter - (3'b001));
      end
    end
  end

endmodule

module BmbArbiter_2_ (
      input   io_inputs_0_cmd_valid,
      output  io_inputs_0_cmd_ready,
      input   io_inputs_0_cmd_payload_last,
      input  [0:0] io_inputs_0_cmd_payload_fragment_source,
      input  [0:0] io_inputs_0_cmd_payload_fragment_opcode,
      input  [23:0] io_inputs_0_cmd_payload_fragment_address,
      input  [4:0] io_inputs_0_cmd_payload_fragment_length,
      input  [0:0] io_inputs_0_cmd_payload_fragment_context,
      output  io_inputs_0_rsp_valid,
      input   io_inputs_0_rsp_ready,
      output  io_inputs_0_rsp_payload_last,
      output [0:0] io_inputs_0_rsp_payload_fragment_source,
      output [0:0] io_inputs_0_rsp_payload_fragment_opcode,
      output [31:0] io_inputs_0_rsp_payload_fragment_data,
      output [0:0] io_inputs_0_rsp_payload_fragment_context,
      output  io_output_cmd_valid,
      input   io_output_cmd_ready,
      output  io_output_cmd_payload_last,
      output [0:0] io_output_cmd_payload_fragment_source,
      output [0:0] io_output_cmd_payload_fragment_opcode,
      output [23:0] io_output_cmd_payload_fragment_address,
      output [4:0] io_output_cmd_payload_fragment_length,
      output [0:0] io_output_cmd_payload_fragment_context,
      input   io_output_rsp_valid,
      output  io_output_rsp_ready,
      input   io_output_rsp_payload_last,
      input  [0:0] io_output_rsp_payload_fragment_source,
      input  [0:0] io_output_rsp_payload_fragment_opcode,
      input  [31:0] io_output_rsp_payload_fragment_data,
      input  [0:0] io_output_rsp_payload_fragment_context);
  assign io_output_cmd_valid = io_inputs_0_cmd_valid;
  assign io_inputs_0_cmd_ready = io_output_cmd_ready;
  assign io_inputs_0_rsp_valid = io_output_rsp_valid;
  assign io_output_rsp_ready = io_inputs_0_rsp_ready;
  assign io_output_cmd_payload_last = io_inputs_0_cmd_payload_last;
  assign io_inputs_0_rsp_payload_last = io_output_rsp_payload_last;
  assign io_output_cmd_payload_fragment_source = io_inputs_0_cmd_payload_fragment_source;
  assign io_output_cmd_payload_fragment_opcode = io_inputs_0_cmd_payload_fragment_opcode;
  assign io_output_cmd_payload_fragment_address = io_inputs_0_cmd_payload_fragment_address;
  assign io_output_cmd_payload_fragment_length = io_inputs_0_cmd_payload_fragment_length;
  assign io_output_cmd_payload_fragment_context = io_inputs_0_cmd_payload_fragment_context;
  assign io_inputs_0_rsp_payload_fragment_source = io_output_rsp_payload_fragment_source;
  assign io_inputs_0_rsp_payload_fragment_opcode = io_output_rsp_payload_fragment_opcode;
  assign io_inputs_0_rsp_payload_fragment_data = io_output_rsp_payload_fragment_data;
  assign io_inputs_0_rsp_payload_fragment_context = io_output_rsp_payload_fragment_context;
endmodule

module BmbDownSizerBridge (
      input   io_input_cmd_valid,
      output  io_input_cmd_ready,
      input   io_input_cmd_payload_last,
      input  [0:0] io_input_cmd_payload_fragment_source,
      input  [0:0] io_input_cmd_payload_fragment_opcode,
      input  [23:0] io_input_cmd_payload_fragment_address,
      input  [4:0] io_input_cmd_payload_fragment_length,
      input  [0:0] io_input_cmd_payload_fragment_context,
      output  io_input_rsp_valid,
      input   io_input_rsp_ready,
      output  io_input_rsp_payload_last,
      output [0:0] io_input_rsp_payload_fragment_source,
      output [0:0] io_input_rsp_payload_fragment_opcode,
      output [31:0] io_input_rsp_payload_fragment_data,
      output [0:0] io_input_rsp_payload_fragment_context,
      output  io_output_cmd_valid,
      input   io_output_cmd_ready,
      output  io_output_cmd_payload_last,
      output [0:0] io_output_cmd_payload_fragment_opcode,
      output [23:0] io_output_cmd_payload_fragment_address,
      output [4:0] io_output_cmd_payload_fragment_length,
      output [3:0] io_output_cmd_payload_fragment_context,
      input   io_output_rsp_valid,
      output  io_output_rsp_ready,
      input   io_output_rsp_payload_last,
      input  [0:0] io_output_rsp_payload_fragment_opcode,
      input  [7:0] io_output_rsp_payload_fragment_data,
      input  [3:0] io_output_rsp_payload_fragment_context,
      input   clk_12M,
      input   clockCtrl_systemReset);
  wire [0:0] cmdContext_context;
  wire [0:0] cmdContext_source;
  wire [1:0] cmdContext_sel;
  wire [0:0] rspContext_context;
  wire [0:0] rspContext_source;
  wire [1:0] rspContext_sel;
  wire [3:0] _zz_1_;
  reg  rspLogic_locked;
  reg [1:0] rspLogic_counter;
  wire [1:0] rspLogic_sel;
  reg [7:0] rspLogic_buffers_0;
  reg [7:0] rspLogic_buffers_1;
  reg [7:0] rspLogic_buffers_2;
  reg [7:0] rspLogic_words_0;
  reg [7:0] rspLogic_words_1;
  reg [7:0] rspLogic_words_2;
  wire [7:0] rspLogic_words_3;
  assign cmdContext_context = io_input_cmd_payload_fragment_context;
  assign cmdContext_source = io_input_cmd_payload_fragment_source;
  assign cmdContext_sel = io_input_cmd_payload_fragment_address[1 : 0];
  assign io_output_cmd_valid = io_input_cmd_valid;
  assign io_output_cmd_payload_fragment_opcode = io_input_cmd_payload_fragment_opcode;
  assign io_output_cmd_payload_fragment_address = io_input_cmd_payload_fragment_address;
  assign io_output_cmd_payload_fragment_length = io_input_cmd_payload_fragment_length;
  assign io_output_cmd_payload_fragment_context = {cmdContext_sel,{cmdContext_source,cmdContext_context}};
  assign io_output_cmd_payload_last = io_input_cmd_payload_last;
  assign io_input_cmd_ready = io_output_cmd_ready;
  assign _zz_1_ = io_output_rsp_payload_fragment_context;
  assign rspContext_context = _zz_1_[0 : 0];
  assign rspContext_source = _zz_1_[1 : 1];
  assign rspContext_sel = _zz_1_[3 : 2];
  assign io_input_rsp_payload_last = io_output_rsp_payload_last;
  assign io_input_rsp_payload_fragment_opcode = io_output_rsp_payload_fragment_opcode;
  assign io_input_rsp_payload_fragment_source = rspContext_source;
  assign io_input_rsp_payload_fragment_context = rspContext_context;
  assign io_output_rsp_ready = io_input_rsp_ready;
  assign rspLogic_sel = (rspLogic_locked ? rspLogic_counter : rspContext_sel);
  always @ (*) begin
    rspLogic_words_0 = rspLogic_buffers_0;
    if((io_input_rsp_payload_last && (rspLogic_sel == (2'b00))))begin
      rspLogic_words_0 = io_output_rsp_payload_fragment_data;
    end
  end

  always @ (*) begin
    rspLogic_words_1 = rspLogic_buffers_1;
    if((io_input_rsp_payload_last && (rspLogic_sel == (2'b01))))begin
      rspLogic_words_1 = io_output_rsp_payload_fragment_data;
    end
  end

  always @ (*) begin
    rspLogic_words_2 = rspLogic_buffers_2;
    if((io_input_rsp_payload_last && (rspLogic_sel == (2'b10))))begin
      rspLogic_words_2 = io_output_rsp_payload_fragment_data;
    end
  end

  assign rspLogic_words_3 = io_output_rsp_payload_fragment_data;
  assign io_input_rsp_valid = (io_output_rsp_valid && (io_output_rsp_payload_last || (rspLogic_sel == (2'b11))));
  assign io_input_rsp_payload_fragment_data = {rspLogic_words_3,{rspLogic_words_2,{rspLogic_words_1,rspLogic_words_0}}};
  always @ (posedge clk_12M) begin
    if(clockCtrl_systemReset) begin
      rspLogic_locked <= 1'b0;
    end else begin
      if((io_output_rsp_valid && io_output_rsp_ready))begin
        rspLogic_locked <= (! io_output_rsp_payload_last);
      end
    end
  end

  always @ (posedge clk_12M) begin
    if((io_output_rsp_valid && io_output_rsp_ready))begin
      rspLogic_counter <= (rspLogic_sel + (2'b01));
      if((rspLogic_sel == (2'b00)))begin
        rspLogic_buffers_0 <= io_output_rsp_payload_fragment_data;
      end
      if((rspLogic_sel == (2'b01)))begin
        rspLogic_buffers_1 <= io_output_rsp_payload_fragment_data;
      end
      if((rspLogic_sel == (2'b10)))begin
        rspLogic_buffers_2 <= io_output_rsp_payload_fragment_data;
      end
    end
  end

endmodule

module Apb3Decoder (
      input  [23:0] io_input_PADDR,
      input  [0:0] io_input_PSEL,
      input   io_input_PENABLE,
      output reg  io_input_PREADY,
      input   io_input_PWRITE,
      input  [31:0] io_input_PWDATA,
      output [31:0] io_input_PRDATA,
      output reg  io_input_PSLVERROR,
      output [23:0] io_output_PADDR,
      output reg [4:0] io_output_PSEL,
      output  io_output_PENABLE,
      input   io_output_PREADY,
      output  io_output_PWRITE,
      output [31:0] io_output_PWDATA,
      input  [31:0] io_output_PRDATA,
      input   io_output_PSLVERROR);
  wire  _zz_1_;
  assign _zz_1_ = (io_input_PSEL[0] && (io_output_PSEL == (5'b00000)));
  assign io_output_PADDR = io_input_PADDR;
  assign io_output_PENABLE = io_input_PENABLE;
  assign io_output_PWRITE = io_input_PWRITE;
  assign io_output_PWDATA = io_input_PWDATA;
  always @ (*) begin
    io_output_PSEL[0] = (((io_input_PADDR & (~ (24'b001111111111111111111111))) == (24'b110000000000000000000000)) && io_input_PSEL[0]);
    io_output_PSEL[1] = (((io_input_PADDR & (~ (24'b000000000000000000001111))) == (24'b000000010000000000000000)) && io_input_PSEL[0]);
    io_output_PSEL[2] = (((io_input_PADDR & (~ (24'b000000000000111111111111))) == (24'b000000000000000000000000)) && io_input_PSEL[0]);
    io_output_PSEL[3] = (((io_input_PADDR & (~ (24'b000000000000000011111111))) == (24'b000000100000000000000000)) && io_input_PSEL[0]);
    io_output_PSEL[4] = (((io_input_PADDR & (~ (24'b000000000000000000001111))) == (24'b000000001000000000000000)) && io_input_PSEL[0]);
  end

  always @ (*) begin
    io_input_PREADY = io_output_PREADY;
    if(_zz_1_)begin
      io_input_PREADY = 1'b1;
    end
  end

  assign io_input_PRDATA = io_output_PRDATA;
  always @ (*) begin
    io_input_PSLVERROR = io_output_PSLVERROR;
    if(_zz_1_)begin
      io_input_PSLVERROR = 1'b1;
    end
  end

endmodule

module Apb3Router (
      input  [23:0] io_input_PADDR,
      input  [4:0] io_input_PSEL,
      input   io_input_PENABLE,
      output  io_input_PREADY,
      input   io_input_PWRITE,
      input  [31:0] io_input_PWDATA,
      output [31:0] io_input_PRDATA,
      output  io_input_PSLVERROR,
      output [23:0] io_outputs_0_PADDR,
      output [0:0] io_outputs_0_PSEL,
      output  io_outputs_0_PENABLE,
      input   io_outputs_0_PREADY,
      output  io_outputs_0_PWRITE,
      output [31:0] io_outputs_0_PWDATA,
      input  [31:0] io_outputs_0_PRDATA,
      input   io_outputs_0_PSLVERROR,
      output [23:0] io_outputs_1_PADDR,
      output [0:0] io_outputs_1_PSEL,
      output  io_outputs_1_PENABLE,
      input   io_outputs_1_PREADY,
      output  io_outputs_1_PWRITE,
      output [31:0] io_outputs_1_PWDATA,
      input  [31:0] io_outputs_1_PRDATA,
      input   io_outputs_1_PSLVERROR,
      output [23:0] io_outputs_2_PADDR,
      output [0:0] io_outputs_2_PSEL,
      output  io_outputs_2_PENABLE,
      input   io_outputs_2_PREADY,
      output  io_outputs_2_PWRITE,
      output [31:0] io_outputs_2_PWDATA,
      input  [31:0] io_outputs_2_PRDATA,
      input   io_outputs_2_PSLVERROR,
      output [23:0] io_outputs_3_PADDR,
      output [0:0] io_outputs_3_PSEL,
      output  io_outputs_3_PENABLE,
      input   io_outputs_3_PREADY,
      output  io_outputs_3_PWRITE,
      output [31:0] io_outputs_3_PWDATA,
      input  [31:0] io_outputs_3_PRDATA,
      input   io_outputs_3_PSLVERROR,
      output [23:0] io_outputs_4_PADDR,
      output [0:0] io_outputs_4_PSEL,
      output  io_outputs_4_PENABLE,
      input   io_outputs_4_PREADY,
      output  io_outputs_4_PWRITE,
      output [31:0] io_outputs_4_PWDATA,
      input  [31:0] io_outputs_4_PRDATA,
      input   io_outputs_4_PSLVERROR,
      input   clk_12M,
      input   clockCtrl_systemReset);
  reg  _zz_5_;
  reg [31:0] _zz_6_;
  reg  _zz_7_;
  wire  _zz_1_;
  wire  _zz_2_;
  wire  _zz_3_;
  wire  _zz_4_;
  reg [2:0] selIndex;
  always @(*) begin
    case(selIndex)
      3'b000 : begin
        _zz_5_ = io_outputs_0_PREADY;
        _zz_6_ = io_outputs_0_PRDATA;
        _zz_7_ = io_outputs_0_PSLVERROR;
      end
      3'b001 : begin
        _zz_5_ = io_outputs_1_PREADY;
        _zz_6_ = io_outputs_1_PRDATA;
        _zz_7_ = io_outputs_1_PSLVERROR;
      end
      3'b010 : begin
        _zz_5_ = io_outputs_2_PREADY;
        _zz_6_ = io_outputs_2_PRDATA;
        _zz_7_ = io_outputs_2_PSLVERROR;
      end
      3'b011 : begin
        _zz_5_ = io_outputs_3_PREADY;
        _zz_6_ = io_outputs_3_PRDATA;
        _zz_7_ = io_outputs_3_PSLVERROR;
      end
      default : begin
        _zz_5_ = io_outputs_4_PREADY;
        _zz_6_ = io_outputs_4_PRDATA;
        _zz_7_ = io_outputs_4_PSLVERROR;
      end
    endcase
  end

  assign io_outputs_0_PADDR = io_input_PADDR;
  assign io_outputs_0_PENABLE = io_input_PENABLE;
  assign io_outputs_0_PSEL[0] = io_input_PSEL[0];
  assign io_outputs_0_PWRITE = io_input_PWRITE;
  assign io_outputs_0_PWDATA = io_input_PWDATA;
  assign io_outputs_1_PADDR = io_input_PADDR;
  assign io_outputs_1_PENABLE = io_input_PENABLE;
  assign io_outputs_1_PSEL[0] = io_input_PSEL[1];
  assign io_outputs_1_PWRITE = io_input_PWRITE;
  assign io_outputs_1_PWDATA = io_input_PWDATA;
  assign io_outputs_2_PADDR = io_input_PADDR;
  assign io_outputs_2_PENABLE = io_input_PENABLE;
  assign io_outputs_2_PSEL[0] = io_input_PSEL[2];
  assign io_outputs_2_PWRITE = io_input_PWRITE;
  assign io_outputs_2_PWDATA = io_input_PWDATA;
  assign io_outputs_3_PADDR = io_input_PADDR;
  assign io_outputs_3_PENABLE = io_input_PENABLE;
  assign io_outputs_3_PSEL[0] = io_input_PSEL[3];
  assign io_outputs_3_PWRITE = io_input_PWRITE;
  assign io_outputs_3_PWDATA = io_input_PWDATA;
  assign io_outputs_4_PADDR = io_input_PADDR;
  assign io_outputs_4_PENABLE = io_input_PENABLE;
  assign io_outputs_4_PSEL[0] = io_input_PSEL[4];
  assign io_outputs_4_PWRITE = io_input_PWRITE;
  assign io_outputs_4_PWDATA = io_input_PWDATA;
  assign _zz_1_ = io_input_PSEL[3];
  assign _zz_2_ = io_input_PSEL[4];
  assign _zz_3_ = (io_input_PSEL[1] || _zz_1_);
  assign _zz_4_ = (io_input_PSEL[2] || _zz_1_);
  assign io_input_PREADY = _zz_5_;
  assign io_input_PRDATA = _zz_6_;
  assign io_input_PSLVERROR = _zz_7_;
  always @ (posedge clk_12M) begin
    selIndex <= {_zz_2_,{_zz_4_,_zz_3_}};
  end

endmodule

module BmbToApb3Bridge (
      input   io_input_cmd_valid,
      output  io_input_cmd_ready,
      input   io_input_cmd_payload_last,
      input  [0:0] io_input_cmd_payload_fragment_source,
      input  [0:0] io_input_cmd_payload_fragment_opcode,
      input  [23:0] io_input_cmd_payload_fragment_address,
      input  [1:0] io_input_cmd_payload_fragment_length,
      input  [31:0] io_input_cmd_payload_fragment_data,
      input  [3:0] io_input_cmd_payload_fragment_mask,
      input  [2:0] io_input_cmd_payload_fragment_context,
      output  io_input_rsp_valid,
      input   io_input_rsp_ready,
      output  io_input_rsp_payload_last,
      output [0:0] io_input_rsp_payload_fragment_source,
      output [0:0] io_input_rsp_payload_fragment_opcode,
      output [31:0] io_input_rsp_payload_fragment_data,
      output [2:0] io_input_rsp_payload_fragment_context,
      output [23:0] io_output_PADDR,
      output [0:0] io_output_PSEL,
      output  io_output_PENABLE,
      input   io_output_PREADY,
      output  io_output_PWRITE,
      output [31:0] io_output_PWDATA,
      input  [31:0] io_output_PRDATA,
      input   io_output_PSLVERROR,
      input   clk_12M,
      input   clockCtrl_systemReset);
  wire  _zz_8_;
  wire  bmbBuffer_cmd_valid;
  reg  bmbBuffer_cmd_ready;
  wire  bmbBuffer_cmd_payload_last;
  wire [0:0] bmbBuffer_cmd_payload_fragment_source;
  wire [0:0] bmbBuffer_cmd_payload_fragment_opcode;
  wire [23:0] bmbBuffer_cmd_payload_fragment_address;
  wire [1:0] bmbBuffer_cmd_payload_fragment_length;
  wire [31:0] bmbBuffer_cmd_payload_fragment_data;
  wire [3:0] bmbBuffer_cmd_payload_fragment_mask;
  wire [2:0] bmbBuffer_cmd_payload_fragment_context;
  reg  bmbBuffer_rsp_valid;
  wire  bmbBuffer_rsp_ready;
  wire  bmbBuffer_rsp_payload_last;
  wire [0:0] bmbBuffer_rsp_payload_fragment_source;
  reg [0:0] bmbBuffer_rsp_payload_fragment_opcode;
  wire [31:0] bmbBuffer_rsp_payload_fragment_data;
  wire [2:0] bmbBuffer_rsp_payload_fragment_context;
  wire  _zz_1_;
  wire  bmbBuffer_rsp_m2sPipe_valid;
  wire  bmbBuffer_rsp_m2sPipe_ready;
  wire  bmbBuffer_rsp_m2sPipe_payload_last;
  wire [0:0] bmbBuffer_rsp_m2sPipe_payload_fragment_source;
  wire [0:0] bmbBuffer_rsp_m2sPipe_payload_fragment_opcode;
  wire [31:0] bmbBuffer_rsp_m2sPipe_payload_fragment_data;
  wire [2:0] bmbBuffer_rsp_m2sPipe_payload_fragment_context;
  reg  _zz_2_;
  reg  _zz_3_;
  reg [0:0] _zz_4_;
  reg [0:0] _zz_5_;
  reg [31:0] _zz_6_;
  reg [2:0] _zz_7_;
  reg  state;
  assign _zz_8_ = (! state);
  assign _zz_1_ = (! (io_input_rsp_valid && (! io_input_rsp_ready)));
  assign io_input_cmd_ready = (bmbBuffer_cmd_ready && _zz_1_);
  assign bmbBuffer_cmd_valid = (io_input_cmd_valid && _zz_1_);
  assign bmbBuffer_cmd_payload_last = io_input_cmd_payload_last;
  assign bmbBuffer_cmd_payload_fragment_source = io_input_cmd_payload_fragment_source;
  assign bmbBuffer_cmd_payload_fragment_opcode = io_input_cmd_payload_fragment_opcode;
  assign bmbBuffer_cmd_payload_fragment_address = io_input_cmd_payload_fragment_address;
  assign bmbBuffer_cmd_payload_fragment_length = io_input_cmd_payload_fragment_length;
  assign bmbBuffer_cmd_payload_fragment_data = io_input_cmd_payload_fragment_data;
  assign bmbBuffer_cmd_payload_fragment_mask = io_input_cmd_payload_fragment_mask;
  assign bmbBuffer_cmd_payload_fragment_context = io_input_cmd_payload_fragment_context;
  assign bmbBuffer_rsp_ready = ((1'b1 && (! bmbBuffer_rsp_m2sPipe_valid)) || bmbBuffer_rsp_m2sPipe_ready);
  assign bmbBuffer_rsp_m2sPipe_valid = _zz_2_;
  assign bmbBuffer_rsp_m2sPipe_payload_last = _zz_3_;
  assign bmbBuffer_rsp_m2sPipe_payload_fragment_source = _zz_4_;
  assign bmbBuffer_rsp_m2sPipe_payload_fragment_opcode = _zz_5_;
  assign bmbBuffer_rsp_m2sPipe_payload_fragment_data = _zz_6_;
  assign bmbBuffer_rsp_m2sPipe_payload_fragment_context = _zz_7_;
  assign io_input_rsp_valid = bmbBuffer_rsp_m2sPipe_valid;
  assign bmbBuffer_rsp_m2sPipe_ready = io_input_rsp_ready;
  assign io_input_rsp_payload_last = bmbBuffer_rsp_m2sPipe_payload_last;
  assign io_input_rsp_payload_fragment_source = bmbBuffer_rsp_m2sPipe_payload_fragment_source;
  assign io_input_rsp_payload_fragment_opcode = bmbBuffer_rsp_m2sPipe_payload_fragment_opcode;
  assign io_input_rsp_payload_fragment_data = bmbBuffer_rsp_m2sPipe_payload_fragment_data;
  assign io_input_rsp_payload_fragment_context = bmbBuffer_rsp_m2sPipe_payload_fragment_context;
  always @ (*) begin
    bmbBuffer_cmd_ready = 1'b0;
    if(! _zz_8_) begin
      if(io_output_PREADY)begin
        bmbBuffer_cmd_ready = 1'b1;
      end
    end
  end

  assign io_output_PSEL[0] = bmbBuffer_cmd_valid;
  assign io_output_PENABLE = state;
  assign io_output_PWRITE = (bmbBuffer_cmd_payload_fragment_opcode == (1'b1));
  assign io_output_PADDR = bmbBuffer_cmd_payload_fragment_address;
  assign io_output_PWDATA = bmbBuffer_cmd_payload_fragment_data;
  always @ (*) begin
    bmbBuffer_rsp_valid = 1'b0;
    if(! _zz_8_) begin
      if(io_output_PREADY)begin
        bmbBuffer_rsp_valid = 1'b1;
      end
    end
  end

  assign bmbBuffer_rsp_payload_fragment_data = io_output_PRDATA;
  assign bmbBuffer_rsp_payload_fragment_source = io_input_cmd_payload_fragment_source;
  assign bmbBuffer_rsp_payload_fragment_context = io_input_cmd_payload_fragment_context;
  assign bmbBuffer_rsp_payload_last = 1'b1;
  always @ (*) begin
    bmbBuffer_rsp_payload_fragment_opcode = (1'b0);
    if(io_output_PSLVERROR)begin
      bmbBuffer_rsp_payload_fragment_opcode = (1'b1);
    end
  end

  always @ (posedge clk_12M) begin
    if(clockCtrl_systemReset) begin
      _zz_2_ <= 1'b0;
      state <= 1'b0;
    end else begin
      if(bmbBuffer_rsp_ready)begin
        _zz_2_ <= bmbBuffer_rsp_valid;
      end
      if(_zz_8_)begin
        state <= bmbBuffer_cmd_valid;
      end else begin
        if(io_output_PREADY)begin
          state <= 1'b0;
        end
      end
    end
  end

  always @ (posedge clk_12M) begin
    if(bmbBuffer_rsp_ready)begin
      _zz_3_ <= bmbBuffer_rsp_payload_last;
      _zz_4_ <= bmbBuffer_rsp_payload_fragment_source;
      _zz_5_ <= bmbBuffer_rsp_payload_fragment_opcode;
      _zz_6_ <= bmbBuffer_rsp_payload_fragment_data;
      _zz_7_ <= bmbBuffer_rsp_payload_fragment_context;
    end
  end

endmodule

module BmbArbiter_3_ (
      input   io_inputs_0_cmd_valid,
      output  io_inputs_0_cmd_ready,
      input   io_inputs_0_cmd_payload_last,
      input  [0:0] io_inputs_0_cmd_payload_fragment_source,
      input  [0:0] io_inputs_0_cmd_payload_fragment_opcode,
      input  [23:0] io_inputs_0_cmd_payload_fragment_address,
      input  [4:0] io_inputs_0_cmd_payload_fragment_length,
      input  [31:0] io_inputs_0_cmd_payload_fragment_data,
      input  [3:0] io_inputs_0_cmd_payload_fragment_mask,
      input  [0:0] io_inputs_0_cmd_payload_fragment_context,
      output  io_inputs_0_rsp_valid,
      input   io_inputs_0_rsp_ready,
      output  io_inputs_0_rsp_payload_last,
      output [0:0] io_inputs_0_rsp_payload_fragment_source,
      output [0:0] io_inputs_0_rsp_payload_fragment_opcode,
      output [31:0] io_inputs_0_rsp_payload_fragment_data,
      output [0:0] io_inputs_0_rsp_payload_fragment_context,
      output  io_output_cmd_valid,
      input   io_output_cmd_ready,
      output  io_output_cmd_payload_last,
      output [0:0] io_output_cmd_payload_fragment_source,
      output [0:0] io_output_cmd_payload_fragment_opcode,
      output [23:0] io_output_cmd_payload_fragment_address,
      output [4:0] io_output_cmd_payload_fragment_length,
      output [31:0] io_output_cmd_payload_fragment_data,
      output [3:0] io_output_cmd_payload_fragment_mask,
      output [0:0] io_output_cmd_payload_fragment_context,
      input   io_output_rsp_valid,
      output  io_output_rsp_ready,
      input   io_output_rsp_payload_last,
      input  [0:0] io_output_rsp_payload_fragment_source,
      input  [0:0] io_output_rsp_payload_fragment_opcode,
      input  [31:0] io_output_rsp_payload_fragment_data,
      input  [0:0] io_output_rsp_payload_fragment_context);
  assign io_output_cmd_valid = io_inputs_0_cmd_valid;
  assign io_inputs_0_cmd_ready = io_output_cmd_ready;
  assign io_inputs_0_rsp_valid = io_output_rsp_valid;
  assign io_output_rsp_ready = io_inputs_0_rsp_ready;
  assign io_output_cmd_payload_last = io_inputs_0_cmd_payload_last;
  assign io_inputs_0_rsp_payload_last = io_output_rsp_payload_last;
  assign io_output_cmd_payload_fragment_source = io_inputs_0_cmd_payload_fragment_source;
  assign io_output_cmd_payload_fragment_opcode = io_inputs_0_cmd_payload_fragment_opcode;
  assign io_output_cmd_payload_fragment_address = io_inputs_0_cmd_payload_fragment_address;
  assign io_output_cmd_payload_fragment_length = io_inputs_0_cmd_payload_fragment_length;
  assign io_output_cmd_payload_fragment_data = io_inputs_0_cmd_payload_fragment_data;
  assign io_output_cmd_payload_fragment_mask = io_inputs_0_cmd_payload_fragment_mask;
  assign io_output_cmd_payload_fragment_context = io_inputs_0_cmd_payload_fragment_context;
  assign io_inputs_0_rsp_payload_fragment_source = io_output_rsp_payload_fragment_source;
  assign io_inputs_0_rsp_payload_fragment_opcode = io_output_rsp_payload_fragment_opcode;
  assign io_inputs_0_rsp_payload_fragment_data = io_output_rsp_payload_fragment_data;
  assign io_inputs_0_rsp_payload_fragment_context = io_output_rsp_payload_fragment_context;
endmodule

module BmbUnburstify_1_ (
      input   io_input_cmd_valid,
      output reg  io_input_cmd_ready,
      input   io_input_cmd_payload_last,
      input  [0:0] io_input_cmd_payload_fragment_source,
      input  [0:0] io_input_cmd_payload_fragment_opcode,
      input  [23:0] io_input_cmd_payload_fragment_address,
      input  [4:0] io_input_cmd_payload_fragment_length,
      input  [31:0] io_input_cmd_payload_fragment_data,
      input  [3:0] io_input_cmd_payload_fragment_mask,
      input  [0:0] io_input_cmd_payload_fragment_context,
      output  io_input_rsp_valid,
      input   io_input_rsp_ready,
      output  io_input_rsp_payload_last,
      output [0:0] io_input_rsp_payload_fragment_source,
      output [0:0] io_input_rsp_payload_fragment_opcode,
      output [31:0] io_input_rsp_payload_fragment_data,
      output [0:0] io_input_rsp_payload_fragment_context,
      output reg  io_output_cmd_valid,
      input   io_output_cmd_ready,
      output  io_output_cmd_payload_last,
      output reg [0:0] io_output_cmd_payload_fragment_source,
      output reg [0:0] io_output_cmd_payload_fragment_opcode,
      output reg [23:0] io_output_cmd_payload_fragment_address,
      output reg [1:0] io_output_cmd_payload_fragment_length,
      output [31:0] io_output_cmd_payload_fragment_data,
      output [3:0] io_output_cmd_payload_fragment_mask,
      output reg [2:0] io_output_cmd_payload_fragment_context,
      input   io_output_rsp_valid,
      output  io_output_rsp_ready,
      input   io_output_rsp_payload_last,
      input  [0:0] io_output_rsp_payload_fragment_source,
      input  [0:0] io_output_rsp_payload_fragment_opcode,
      input  [31:0] io_output_rsp_payload_fragment_data,
      input  [2:0] io_output_rsp_payload_fragment_context,
      input   clk_12M,
      input   clockCtrl_systemReset);
  wire  _zz_1_;
  wire [11:0] _zz_2_;
  wire [11:0] _zz_3_;
  wire [11:0] _zz_4_;
  wire  doResult;
  reg  buffer_valid;
  reg [0:0] buffer_opcode;
  reg [0:0] buffer_source;
  reg [23:0] buffer_address;
  reg [0:0] buffer_context;
  reg [2:0] buffer_beat;
  wire  buffer_last;
  wire [23:0] buffer_addressIncr;
  wire  buffer_isWrite;
  wire [2:0] cmdTransferBeatCount;
  wire  requireBuffer;
  assign _zz_1_ = (io_output_cmd_valid && io_output_cmd_ready);
  assign _zz_2_ = (_zz_4_ + (12'b000000000100));
  assign _zz_3_ = buffer_address[11 : 0];
  assign _zz_4_ = _zz_3_;
  assign buffer_last = (buffer_beat == (3'b001));
  assign buffer_addressIncr = {buffer_address[23 : 12],(_zz_2_ & (~ (12'b000000000011)))};
  assign buffer_isWrite = (buffer_opcode == (1'b1));
  assign cmdTransferBeatCount = io_input_cmd_payload_fragment_length[4 : 2];
  assign requireBuffer = (cmdTransferBeatCount != (3'b000));
  assign io_output_cmd_payload_fragment_data = io_input_cmd_payload_fragment_data;
  assign io_output_cmd_payload_fragment_mask = io_input_cmd_payload_fragment_mask;
  assign io_output_cmd_payload_last = 1'b1;
  always @ (*) begin
    if(buffer_valid)begin
      io_output_cmd_payload_fragment_source = buffer_source;
    end else begin
      io_output_cmd_payload_fragment_source = io_input_cmd_payload_fragment_source;
    end
  end

  always @ (*) begin
    if(buffer_valid)begin
      io_output_cmd_payload_fragment_address = buffer_addressIncr;
    end else begin
      io_output_cmd_payload_fragment_address = io_input_cmd_payload_fragment_address;
      if(requireBuffer)begin
        io_output_cmd_payload_fragment_address[1 : 0] = (2'b00);
      end
    end
  end

  always @ (*) begin
    if(buffer_valid)begin
      io_output_cmd_payload_fragment_opcode = buffer_opcode;
    end else begin
      io_output_cmd_payload_fragment_opcode = io_input_cmd_payload_fragment_opcode;
    end
  end

  always @ (*) begin
    if(buffer_valid)begin
      io_output_cmd_payload_fragment_length = (2'b11);
    end else begin
      if(requireBuffer)begin
        io_output_cmd_payload_fragment_length = (2'b11);
      end else begin
        io_output_cmd_payload_fragment_length = io_input_cmd_payload_fragment_length[1:0];
      end
    end
  end

  always @ (*) begin
    if(buffer_valid)begin
      io_output_cmd_payload_fragment_context[0 : 0] = buffer_context;
    end else begin
      io_output_cmd_payload_fragment_context[0 : 0] = io_input_cmd_payload_fragment_context;
    end
    if(buffer_valid)begin
      io_output_cmd_payload_fragment_context[2] = buffer_last;
      io_output_cmd_payload_fragment_context[1] = buffer_isWrite;
    end else begin
      io_output_cmd_payload_fragment_context[1] = (io_input_cmd_payload_fragment_opcode == (1'b1));
      io_output_cmd_payload_fragment_context[2] = (! requireBuffer);
    end
  end

  always @ (*) begin
    io_input_cmd_ready = 1'b0;
    if(buffer_valid)begin
      io_input_cmd_ready = (buffer_isWrite && io_output_cmd_ready);
    end else begin
      io_input_cmd_ready = io_output_cmd_ready;
    end
  end

  always @ (*) begin
    if(buffer_valid)begin
      io_output_cmd_valid = (! (buffer_isWrite && (! io_input_cmd_valid)));
    end else begin
      io_output_cmd_valid = io_input_cmd_valid;
    end
  end

  assign io_input_rsp_valid = (io_output_rsp_valid && (io_output_rsp_payload_fragment_context[2] || (! io_output_rsp_payload_fragment_context[1])));
  assign io_input_rsp_payload_last = io_output_rsp_payload_fragment_context[2];
  assign io_input_rsp_payload_fragment_source = io_output_rsp_payload_fragment_source;
  assign io_input_rsp_payload_fragment_opcode = io_output_rsp_payload_fragment_opcode;
  assign io_input_rsp_payload_fragment_data = io_output_rsp_payload_fragment_data;
  assign io_input_rsp_payload_fragment_context = io_output_rsp_payload_fragment_context[0:0];
  assign io_output_rsp_ready = io_input_rsp_ready;
  always @ (posedge clk_12M) begin
    if(clockCtrl_systemReset) begin
      buffer_valid <= 1'b0;
    end else begin
      if(_zz_1_)begin
        if(buffer_last)begin
          buffer_valid <= 1'b0;
        end
      end
      if(! buffer_valid) begin
        buffer_valid <= (requireBuffer && (io_output_cmd_valid && io_output_cmd_ready));
      end
    end
  end

  always @ (posedge clk_12M) begin
    if(_zz_1_)begin
      buffer_beat <= (buffer_beat - (3'b001));
      buffer_address[11 : 0] <= buffer_addressIncr[11 : 0];
    end
    if(! buffer_valid) begin
      buffer_opcode <= io_input_cmd_payload_fragment_opcode;
      buffer_source <= io_input_cmd_payload_fragment_source;
      buffer_address <= io_input_cmd_payload_fragment_address;
      buffer_context <= io_input_cmd_payload_fragment_context;
      buffer_beat <= cmdTransferBeatCount;
    end
  end

endmodule

module Ice40up5kbevn (
      input   clk_12M,
      output  system_uartA_uart_txd,
      input   system_uartA_uart_rxd,
      input   system_cpu_tms,
      input   system_cpu_tdi,
      output  system_cpu_tdo,
      input   system_cpu_tck,
      inout  system_spiA_spi_sclk,
      inout  system_spiA_spi_ss_0,
      inout  system_spiA_spi_data_0,
      inout  system_spiA_spi_data_1,
      inout  system_gpioA_gpio_0,
      inout  system_gpioA_gpio_1,
      inout  system_gpioA_gpio_2,
      inout  system_gpioA_gpio_3);
  wire [3:0] _zz_35_;
  wire [11:0] _zz_36_;
  wire [3:0] _zz_37_;
  wire  _zz_38_;
  wire [7:0] _zz_39_;
  wire  _zz_40_;
  wire  _zz_41_;
  wire  _zz_42_;
  wire  _zz_43_;
  wire [0:0] _zz_44_;
  wire [4:0] _zz_45_;
  wire  _zz_46_;
  wire  _zz_47_;
  wire [0:0] _zz_48_;
  wire [3:0] _zz_49_;
  wire  _zz_50_;
  wire [7:0] _zz_51_;
  wire  _zz_52_;
  wire  _zz_53_;
  wire  _zz_54_;
  wire  _zz_55_;
  wire  _zz_56_;
  wire  _zz_57_;
  wire  _zz_58_;
  wire  _zz_59_;
  wire  _zz_60_;
  wire  _zz_61_;
  wire  _zz_62_;
  wire  _zz_63_;
  wire [4:0] _zz_64_;
  wire [31:0] _zz_65_;
  wire [3:0] _zz_66_;
  wire [0:0] _zz_67_;
  wire [16:0] _zz_68_;
  wire [23:0] _zz_69_;
  wire  _zz_70_;
  wire  _zz_71_;
  wire [23:0] _zz_72_;
  wire  _zz_73_;
  wire  _zz_74_;
  wire  _zz_75_;
  wire  _zz_76_;
  wire  _zz_77_;
  wire  _zz_78_;
  wire  _zz_79_;
  wire  _zz_80_;
  wire  bufferCC_4__io_dataOut;
  wire  system_uartA_io_apb_PREADY;
  wire [31:0] system_uartA_io_apb_PRDATA;
  wire  system_uartA_io_uart_txd;
  wire  system_uartA_io_interrupt;
  wire [3:0] system_gpioA_io_gpio_write;
  wire [3:0] system_gpioA_io_gpio_writeEnable;
  wire  system_gpioA_io_bus_PREADY;
  wire [31:0] system_gpioA_io_bus_PRDATA;
  wire  system_gpioA_io_bus_PSLVERROR;
  wire [3:0] system_gpioA_io_interrupt;
  wire  system_machineTimer_io_bus_PREADY;
  wire [31:0] system_machineTimer_io_bus_PRDATA;
  wire  system_machineTimer_io_bus_PSLVERROR;
  wire  system_machineTimer_io_mTimeInterrupt;
  wire  system_cpu_cpu_debug_bus_cmd_ready;
  wire [31:0] system_cpu_cpu_debug_bus_rsp_data;
  wire  system_cpu_cpu_debug_resetOut;
  wire  system_cpu_cpu_iBus_cmd_valid;
  wire [31:0] system_cpu_cpu_iBus_cmd_payload_address;
  wire [2:0] system_cpu_cpu_iBus_cmd_payload_size;
  wire  system_cpu_cpu_dBus_cmd_valid;
  wire  system_cpu_cpu_dBus_cmd_payload_wr;
  wire [31:0] system_cpu_cpu_dBus_cmd_payload_address;
  wire [31:0] system_cpu_cpu_dBus_cmd_payload_data;
  wire [1:0] system_cpu_cpu_dBus_cmd_payload_size;
  wire  jtagBridge_1__io_jtag_tdo;
  wire  jtagBridge_1__io_remote_cmd_valid;
  wire  jtagBridge_1__io_remote_cmd_payload_last;
  wire [0:0] jtagBridge_1__io_remote_cmd_payload_fragment;
  wire  jtagBridge_1__io_remote_rsp_ready;
  wire  systemDebugger_1__io_remote_cmd_ready;
  wire  systemDebugger_1__io_remote_rsp_valid;
  wire  systemDebugger_1__io_remote_rsp_payload_error;
  wire [31:0] systemDebugger_1__io_remote_rsp_payload_data;
  wire  systemDebugger_1__io_mem_cmd_valid;
  wire [31:0] systemDebugger_1__io_mem_cmd_payload_address;
  wire [31:0] systemDebugger_1__io_mem_cmd_payload_data;
  wire  systemDebugger_1__io_mem_cmd_payload_wr;
  wire [1:0] systemDebugger_1__io_mem_cmd_payload_size;
  wire  system_cpu_iBus_decoder_io_input_cmd_ready;
  wire  system_cpu_iBus_decoder_io_input_rsp_valid;
  wire  system_cpu_iBus_decoder_io_input_rsp_payload_last;
  wire [0:0] system_cpu_iBus_decoder_io_input_rsp_payload_fragment_opcode;
  wire [31:0] system_cpu_iBus_decoder_io_input_rsp_payload_fragment_data;
  wire  system_cpu_iBus_decoder_io_outputs_0_cmd_valid;
  wire  system_cpu_iBus_decoder_io_outputs_0_cmd_payload_last;
  wire [0:0] system_cpu_iBus_decoder_io_outputs_0_cmd_payload_fragment_opcode;
  wire [31:0] system_cpu_iBus_decoder_io_outputs_0_cmd_payload_fragment_address;
  wire [4:0] system_cpu_iBus_decoder_io_outputs_0_cmd_payload_fragment_length;
  wire  system_cpu_iBus_decoder_io_outputs_0_rsp_ready;
  wire  system_cpu_dBus_decoder_io_input_cmd_ready;
  wire  system_cpu_dBus_decoder_io_input_rsp_valid;
  wire  system_cpu_dBus_decoder_io_input_rsp_payload_last;
  wire [0:0] system_cpu_dBus_decoder_io_input_rsp_payload_fragment_opcode;
  wire [31:0] system_cpu_dBus_decoder_io_input_rsp_payload_fragment_data;
  wire [0:0] system_cpu_dBus_decoder_io_input_rsp_payload_fragment_context;
  wire  system_cpu_dBus_decoder_io_outputs_0_cmd_valid;
  wire  system_cpu_dBus_decoder_io_outputs_0_cmd_payload_last;
  wire [0:0] system_cpu_dBus_decoder_io_outputs_0_cmd_payload_fragment_opcode;
  wire [31:0] system_cpu_dBus_decoder_io_outputs_0_cmd_payload_fragment_address;
  wire [1:0] system_cpu_dBus_decoder_io_outputs_0_cmd_payload_fragment_length;
  wire [31:0] system_cpu_dBus_decoder_io_outputs_0_cmd_payload_fragment_data;
  wire [3:0] system_cpu_dBus_decoder_io_outputs_0_cmd_payload_fragment_mask;
  wire [0:0] system_cpu_dBus_decoder_io_outputs_0_cmd_payload_fragment_context;
  wire  system_cpu_dBus_decoder_io_outputs_0_rsp_ready;
  wire  system_spiA_io_apb_PREADY;
  wire [31:0] system_spiA_io_apb_PRDATA;
  wire  system_spiA_io_xip_cmd_ready;
  wire  system_spiA_io_xip_rsp_valid;
  wire  system_spiA_io_xip_rsp_payload_last;
  wire [7:0] system_spiA_io_xip_rsp_payload_fragment;
  wire [1:0] system_spiA_io_spi_sclk_write;
  wire [0:0] system_spiA_io_spi_ss;
  wire [1:0] system_spiA_io_spi_data_0_write;
  wire  system_spiA_io_spi_data_0_writeEnable;
  wire [1:0] system_spiA_io_spi_data_1_write;
  wire  system_spiA_io_spi_data_1_writeEnable;
  wire  system_spiA_io_interrupt;
  wire  sB_IO_1__D_IN_0;
  wire  sB_IO_1__D_IN_1;
  wire  sB_IO_2__D_IN_0;
  wire  sB_IO_2__D_IN_1;
  wire  sB_IO_3__D_IN_0;
  wire  sB_IO_3__D_IN_1;
  wire  sB_IO_4__D_IN_0;
  wire  sB_IO_4__D_IN_1;
  wire  system_bridge_bmb_arbiter_io_inputs_0_cmd_ready;
  wire  system_bridge_bmb_arbiter_io_inputs_0_rsp_valid;
  wire  system_bridge_bmb_arbiter_io_inputs_0_rsp_payload_last;
  wire [0:0] system_bridge_bmb_arbiter_io_inputs_0_rsp_payload_fragment_opcode;
  wire [31:0] system_bridge_bmb_arbiter_io_inputs_0_rsp_payload_fragment_data;
  wire [0:0] system_bridge_bmb_arbiter_io_inputs_0_rsp_payload_fragment_context;
  wire  system_bridge_bmb_arbiter_io_inputs_1_cmd_ready;
  wire  system_bridge_bmb_arbiter_io_inputs_1_rsp_valid;
  wire  system_bridge_bmb_arbiter_io_inputs_1_rsp_payload_last;
  wire [0:0] system_bridge_bmb_arbiter_io_inputs_1_rsp_payload_fragment_opcode;
  wire [31:0] system_bridge_bmb_arbiter_io_inputs_1_rsp_payload_fragment_data;
  wire [0:0] system_bridge_bmb_arbiter_io_inputs_1_rsp_payload_fragment_context;
  wire  system_bridge_bmb_arbiter_io_output_cmd_valid;
  wire  system_bridge_bmb_arbiter_io_output_cmd_payload_last;
  wire [0:0] system_bridge_bmb_arbiter_io_output_cmd_payload_fragment_source;
  wire [0:0] system_bridge_bmb_arbiter_io_output_cmd_payload_fragment_opcode;
  wire [31:0] system_bridge_bmb_arbiter_io_output_cmd_payload_fragment_address;
  wire [4:0] system_bridge_bmb_arbiter_io_output_cmd_payload_fragment_length;
  wire [31:0] system_bridge_bmb_arbiter_io_output_cmd_payload_fragment_data;
  wire [3:0] system_bridge_bmb_arbiter_io_output_cmd_payload_fragment_mask;
  wire [0:0] system_bridge_bmb_arbiter_io_output_cmd_payload_fragment_context;
  wire  system_bridge_bmb_arbiter_io_output_rsp_ready;
  wire  system_ramA_io_bus_cmd_ready;
  wire  system_ramA_io_bus_rsp_valid;
  wire  system_ramA_io_bus_rsp_payload_last;
  wire [0:0] system_ramA_io_bus_rsp_payload_fragment_source;
  wire [0:0] system_ramA_io_bus_rsp_payload_fragment_opcode;
  wire [31:0] system_ramA_io_bus_rsp_payload_fragment_data;
  wire [2:0] system_ramA_io_bus_rsp_payload_fragment_context;
  wire  system_ramA_bmb_arbiter_io_inputs_0_cmd_ready;
  wire  system_ramA_bmb_arbiter_io_inputs_0_rsp_valid;
  wire  system_ramA_bmb_arbiter_io_inputs_0_rsp_payload_last;
  wire [0:0] system_ramA_bmb_arbiter_io_inputs_0_rsp_payload_fragment_source;
  wire [0:0] system_ramA_bmb_arbiter_io_inputs_0_rsp_payload_fragment_opcode;
  wire [31:0] system_ramA_bmb_arbiter_io_inputs_0_rsp_payload_fragment_data;
  wire [0:0] system_ramA_bmb_arbiter_io_inputs_0_rsp_payload_fragment_context;
  wire  system_ramA_bmb_arbiter_io_output_cmd_valid;
  wire  system_ramA_bmb_arbiter_io_output_cmd_payload_last;
  wire [0:0] system_ramA_bmb_arbiter_io_output_cmd_payload_fragment_source;
  wire [0:0] system_ramA_bmb_arbiter_io_output_cmd_payload_fragment_opcode;
  wire [16:0] system_ramA_bmb_arbiter_io_output_cmd_payload_fragment_address;
  wire [4:0] system_ramA_bmb_arbiter_io_output_cmd_payload_fragment_length;
  wire [31:0] system_ramA_bmb_arbiter_io_output_cmd_payload_fragment_data;
  wire [3:0] system_ramA_bmb_arbiter_io_output_cmd_payload_fragment_mask;
  wire [0:0] system_ramA_bmb_arbiter_io_output_cmd_payload_fragment_context;
  wire  system_ramA_bmb_arbiter_io_output_rsp_ready;
  wire  system_ramA_bmb_burstUnburstifier_io_input_cmd_ready;
  wire  system_ramA_bmb_burstUnburstifier_io_input_rsp_valid;
  wire  system_ramA_bmb_burstUnburstifier_io_input_rsp_payload_last;
  wire [0:0] system_ramA_bmb_burstUnburstifier_io_input_rsp_payload_fragment_source;
  wire [0:0] system_ramA_bmb_burstUnburstifier_io_input_rsp_payload_fragment_opcode;
  wire [31:0] system_ramA_bmb_burstUnburstifier_io_input_rsp_payload_fragment_data;
  wire [0:0] system_ramA_bmb_burstUnburstifier_io_input_rsp_payload_fragment_context;
  wire  system_ramA_bmb_burstUnburstifier_io_output_cmd_valid;
  wire  system_ramA_bmb_burstUnburstifier_io_output_cmd_payload_last;
  wire [0:0] system_ramA_bmb_burstUnburstifier_io_output_cmd_payload_fragment_source;
  wire [0:0] system_ramA_bmb_burstUnburstifier_io_output_cmd_payload_fragment_opcode;
  wire [16:0] system_ramA_bmb_burstUnburstifier_io_output_cmd_payload_fragment_address;
  wire [1:0] system_ramA_bmb_burstUnburstifier_io_output_cmd_payload_fragment_length;
  wire [31:0] system_ramA_bmb_burstUnburstifier_io_output_cmd_payload_fragment_data;
  wire [3:0] system_ramA_bmb_burstUnburstifier_io_output_cmd_payload_fragment_mask;
  wire [2:0] system_ramA_bmb_burstUnburstifier_io_output_cmd_payload_fragment_context;
  wire  system_ramA_bmb_burstUnburstifier_io_output_rsp_ready;
  wire  system_bridge_bmb_decoder_io_input_cmd_ready;
  wire  system_bridge_bmb_decoder_io_input_rsp_valid;
  wire  system_bridge_bmb_decoder_io_input_rsp_payload_last;
  wire [0:0] system_bridge_bmb_decoder_io_input_rsp_payload_fragment_source;
  wire [0:0] system_bridge_bmb_decoder_io_input_rsp_payload_fragment_opcode;
  wire [31:0] system_bridge_bmb_decoder_io_input_rsp_payload_fragment_data;
  wire [0:0] system_bridge_bmb_decoder_io_input_rsp_payload_fragment_context;
  wire  system_bridge_bmb_decoder_io_outputs_0_cmd_valid;
  wire  system_bridge_bmb_decoder_io_outputs_0_cmd_payload_last;
  wire [0:0] system_bridge_bmb_decoder_io_outputs_0_cmd_payload_fragment_source;
  wire [0:0] system_bridge_bmb_decoder_io_outputs_0_cmd_payload_fragment_opcode;
  wire [31:0] system_bridge_bmb_decoder_io_outputs_0_cmd_payload_fragment_address;
  wire [4:0] system_bridge_bmb_decoder_io_outputs_0_cmd_payload_fragment_length;
  wire [31:0] system_bridge_bmb_decoder_io_outputs_0_cmd_payload_fragment_data;
  wire [3:0] system_bridge_bmb_decoder_io_outputs_0_cmd_payload_fragment_mask;
  wire [0:0] system_bridge_bmb_decoder_io_outputs_0_cmd_payload_fragment_context;
  wire  system_bridge_bmb_decoder_io_outputs_0_rsp_ready;
  wire  system_bridge_bmb_decoder_io_outputs_1_cmd_valid;
  wire  system_bridge_bmb_decoder_io_outputs_1_cmd_payload_last;
  wire [0:0] system_bridge_bmb_decoder_io_outputs_1_cmd_payload_fragment_source;
  wire [0:0] system_bridge_bmb_decoder_io_outputs_1_cmd_payload_fragment_opcode;
  wire [31:0] system_bridge_bmb_decoder_io_outputs_1_cmd_payload_fragment_address;
  wire [4:0] system_bridge_bmb_decoder_io_outputs_1_cmd_payload_fragment_length;
  wire [31:0] system_bridge_bmb_decoder_io_outputs_1_cmd_payload_fragment_data;
  wire [3:0] system_bridge_bmb_decoder_io_outputs_1_cmd_payload_fragment_mask;
  wire [0:0] system_bridge_bmb_decoder_io_outputs_1_cmd_payload_fragment_context;
  wire  system_bridge_bmb_decoder_io_outputs_1_rsp_ready;
  wire  system_bridge_bmb_decoder_io_outputs_2_cmd_valid;
  wire  system_bridge_bmb_decoder_io_outputs_2_cmd_payload_last;
  wire [0:0] system_bridge_bmb_decoder_io_outputs_2_cmd_payload_fragment_source;
  wire [0:0] system_bridge_bmb_decoder_io_outputs_2_cmd_payload_fragment_opcode;
  wire [31:0] system_bridge_bmb_decoder_io_outputs_2_cmd_payload_fragment_address;
  wire [4:0] system_bridge_bmb_decoder_io_outputs_2_cmd_payload_fragment_length;
  wire [31:0] system_bridge_bmb_decoder_io_outputs_2_cmd_payload_fragment_data;
  wire [3:0] system_bridge_bmb_decoder_io_outputs_2_cmd_payload_fragment_mask;
  wire [0:0] system_bridge_bmb_decoder_io_outputs_2_cmd_payload_fragment_context;
  wire  system_bridge_bmb_decoder_io_outputs_2_rsp_ready;
  wire  system_spiA_bmb_arbiter_io_inputs_0_cmd_ready;
  wire  system_spiA_bmb_arbiter_io_inputs_0_rsp_valid;
  wire  system_spiA_bmb_arbiter_io_inputs_0_rsp_payload_last;
  wire [0:0] system_spiA_bmb_arbiter_io_inputs_0_rsp_payload_fragment_source;
  wire [0:0] system_spiA_bmb_arbiter_io_inputs_0_rsp_payload_fragment_opcode;
  wire [31:0] system_spiA_bmb_arbiter_io_inputs_0_rsp_payload_fragment_data;
  wire [0:0] system_spiA_bmb_arbiter_io_inputs_0_rsp_payload_fragment_context;
  wire  system_spiA_bmb_arbiter_io_output_cmd_valid;
  wire  system_spiA_bmb_arbiter_io_output_cmd_payload_last;
  wire [0:0] system_spiA_bmb_arbiter_io_output_cmd_payload_fragment_source;
  wire [0:0] system_spiA_bmb_arbiter_io_output_cmd_payload_fragment_opcode;
  wire [23:0] system_spiA_bmb_arbiter_io_output_cmd_payload_fragment_address;
  wire [4:0] system_spiA_bmb_arbiter_io_output_cmd_payload_fragment_length;
  wire [0:0] system_spiA_bmb_arbiter_io_output_cmd_payload_fragment_context;
  wire  system_spiA_bmb_arbiter_io_output_rsp_ready;
  wire  system_spiA_bmb_downSizer_io_input_cmd_ready;
  wire  system_spiA_bmb_downSizer_io_input_rsp_valid;
  wire  system_spiA_bmb_downSizer_io_input_rsp_payload_last;
  wire [0:0] system_spiA_bmb_downSizer_io_input_rsp_payload_fragment_source;
  wire [0:0] system_spiA_bmb_downSizer_io_input_rsp_payload_fragment_opcode;
  wire [31:0] system_spiA_bmb_downSizer_io_input_rsp_payload_fragment_data;
  wire [0:0] system_spiA_bmb_downSizer_io_input_rsp_payload_fragment_context;
  wire  system_spiA_bmb_downSizer_io_output_cmd_valid;
  wire  system_spiA_bmb_downSizer_io_output_cmd_payload_last;
  wire [0:0] system_spiA_bmb_downSizer_io_output_cmd_payload_fragment_opcode;
  wire [23:0] system_spiA_bmb_downSizer_io_output_cmd_payload_fragment_address;
  wire [4:0] system_spiA_bmb_downSizer_io_output_cmd_payload_fragment_length;
  wire [3:0] system_spiA_bmb_downSizer_io_output_cmd_payload_fragment_context;
  wire  system_spiA_bmb_downSizer_io_output_rsp_ready;
  wire  generator_decoder_io_input_PREADY;
  wire [31:0] generator_decoder_io_input_PRDATA;
  wire  generator_decoder_io_input_PSLVERROR;
  wire [23:0] generator_decoder_io_output_PADDR;
  wire [4:0] generator_decoder_io_output_PSEL;
  wire  generator_decoder_io_output_PENABLE;
  wire  generator_decoder_io_output_PWRITE;
  wire [31:0] generator_decoder_io_output_PWDATA;
  wire  apb3Router_1__io_input_PREADY;
  wire [31:0] apb3Router_1__io_input_PRDATA;
  wire  apb3Router_1__io_input_PSLVERROR;
  wire [23:0] apb3Router_1__io_outputs_0_PADDR;
  wire [0:0] apb3Router_1__io_outputs_0_PSEL;
  wire  apb3Router_1__io_outputs_0_PENABLE;
  wire  apb3Router_1__io_outputs_0_PWRITE;
  wire [31:0] apb3Router_1__io_outputs_0_PWDATA;
  wire [23:0] apb3Router_1__io_outputs_1_PADDR;
  wire [0:0] apb3Router_1__io_outputs_1_PSEL;
  wire  apb3Router_1__io_outputs_1_PENABLE;
  wire  apb3Router_1__io_outputs_1_PWRITE;
  wire [31:0] apb3Router_1__io_outputs_1_PWDATA;
  wire [23:0] apb3Router_1__io_outputs_2_PADDR;
  wire [0:0] apb3Router_1__io_outputs_2_PSEL;
  wire  apb3Router_1__io_outputs_2_PENABLE;
  wire  apb3Router_1__io_outputs_2_PWRITE;
  wire [31:0] apb3Router_1__io_outputs_2_PWDATA;
  wire [23:0] apb3Router_1__io_outputs_3_PADDR;
  wire [0:0] apb3Router_1__io_outputs_3_PSEL;
  wire  apb3Router_1__io_outputs_3_PENABLE;
  wire  apb3Router_1__io_outputs_3_PWRITE;
  wire [31:0] apb3Router_1__io_outputs_3_PWDATA;
  wire [23:0] apb3Router_1__io_outputs_4_PADDR;
  wire [0:0] apb3Router_1__io_outputs_4_PSEL;
  wire  apb3Router_1__io_outputs_4_PENABLE;
  wire  apb3Router_1__io_outputs_4_PWRITE;
  wire [31:0] apb3Router_1__io_outputs_4_PWDATA;
  wire  system_peripheralBridge_bridge_io_input_cmd_ready;
  wire  system_peripheralBridge_bridge_io_input_rsp_valid;
  wire  system_peripheralBridge_bridge_io_input_rsp_payload_last;
  wire [0:0] system_peripheralBridge_bridge_io_input_rsp_payload_fragment_source;
  wire [0:0] system_peripheralBridge_bridge_io_input_rsp_payload_fragment_opcode;
  wire [31:0] system_peripheralBridge_bridge_io_input_rsp_payload_fragment_data;
  wire [2:0] system_peripheralBridge_bridge_io_input_rsp_payload_fragment_context;
  wire [23:0] system_peripheralBridge_bridge_io_output_PADDR;
  wire [0:0] system_peripheralBridge_bridge_io_output_PSEL;
  wire  system_peripheralBridge_bridge_io_output_PENABLE;
  wire  system_peripheralBridge_bridge_io_output_PWRITE;
  wire [31:0] system_peripheralBridge_bridge_io_output_PWDATA;
  wire  system_peripheralBridge_input_arbiter_io_inputs_0_cmd_ready;
  wire  system_peripheralBridge_input_arbiter_io_inputs_0_rsp_valid;
  wire  system_peripheralBridge_input_arbiter_io_inputs_0_rsp_payload_last;
  wire [0:0] system_peripheralBridge_input_arbiter_io_inputs_0_rsp_payload_fragment_source;
  wire [0:0] system_peripheralBridge_input_arbiter_io_inputs_0_rsp_payload_fragment_opcode;
  wire [31:0] system_peripheralBridge_input_arbiter_io_inputs_0_rsp_payload_fragment_data;
  wire [0:0] system_peripheralBridge_input_arbiter_io_inputs_0_rsp_payload_fragment_context;
  wire  system_peripheralBridge_input_arbiter_io_output_cmd_valid;
  wire  system_peripheralBridge_input_arbiter_io_output_cmd_payload_last;
  wire [0:0] system_peripheralBridge_input_arbiter_io_output_cmd_payload_fragment_source;
  wire [0:0] system_peripheralBridge_input_arbiter_io_output_cmd_payload_fragment_opcode;
  wire [23:0] system_peripheralBridge_input_arbiter_io_output_cmd_payload_fragment_address;
  wire [4:0] system_peripheralBridge_input_arbiter_io_output_cmd_payload_fragment_length;
  wire [31:0] system_peripheralBridge_input_arbiter_io_output_cmd_payload_fragment_data;
  wire [3:0] system_peripheralBridge_input_arbiter_io_output_cmd_payload_fragment_mask;
  wire [0:0] system_peripheralBridge_input_arbiter_io_output_cmd_payload_fragment_context;
  wire  system_peripheralBridge_input_arbiter_io_output_rsp_ready;
  wire  system_peripheralBridge_input_burstUnburstifier_io_input_cmd_ready;
  wire  system_peripheralBridge_input_burstUnburstifier_io_input_rsp_valid;
  wire  system_peripheralBridge_input_burstUnburstifier_io_input_rsp_payload_last;
  wire [0:0] system_peripheralBridge_input_burstUnburstifier_io_input_rsp_payload_fragment_source;
  wire [0:0] system_peripheralBridge_input_burstUnburstifier_io_input_rsp_payload_fragment_opcode;
  wire [31:0] system_peripheralBridge_input_burstUnburstifier_io_input_rsp_payload_fragment_data;
  wire [0:0] system_peripheralBridge_input_burstUnburstifier_io_input_rsp_payload_fragment_context;
  wire  system_peripheralBridge_input_burstUnburstifier_io_output_cmd_valid;
  wire  system_peripheralBridge_input_burstUnburstifier_io_output_cmd_payload_last;
  wire [0:0] system_peripheralBridge_input_burstUnburstifier_io_output_cmd_payload_fragment_source;
  wire [0:0] system_peripheralBridge_input_burstUnburstifier_io_output_cmd_payload_fragment_opcode;
  wire [23:0] system_peripheralBridge_input_burstUnburstifier_io_output_cmd_payload_fragment_address;
  wire [1:0] system_peripheralBridge_input_burstUnburstifier_io_output_cmd_payload_fragment_length;
  wire [31:0] system_peripheralBridge_input_burstUnburstifier_io_output_cmd_payload_fragment_data;
  wire [3:0] system_peripheralBridge_input_burstUnburstifier_io_output_cmd_payload_fragment_mask;
  wire [2:0] system_peripheralBridge_input_burstUnburstifier_io_output_cmd_payload_fragment_context;
  wire  system_peripheralBridge_input_burstUnburstifier_io_output_rsp_ready;
  wire  sB_IO_5__D_IN_0;
  wire  sB_IO_6__D_IN_0;
  wire  sB_IO_7__D_IN_0;
  wire  sB_IO_8__D_IN_0;
  wire  _zz_81_;
  wire [0:0] _zz_82_;
  wire [0:0] _zz_83_;
  wire [0:0] _zz_84_;
  wire  clockCtrl_inputResetTrigger;
  reg  clockCtrl_resetUnbuffered;
  reg [7:0] clockCtrl_holdingLogic_resetCounter = (8'b00000000);
  reg  clockCtrl_resetRequest;
  reg  clockCtrl_systemReset;
  wire [1:0] system_plic_targets_0_gateways_0_priority;
  reg  system_plic_targets_0_gateways_0_ip;
  reg  system_plic_targets_0_gateways_0_waitCompletion;
  reg [3:0] _zz_1_;
  wire [3:0] _zz_2_;
  wire [3:0] _zz_3_;
  reg  clockCtrl_resetUnbuffered_regNext;
  wire [0:0] _zz_4_;
  reg [1:0] _zz_5_;
  reg [3:0] _zz_6_;
  reg  system_cpu_cpu_debug_resetOut_regNext;
  reg  _zz_7_;
  wire [1:0] system_spiA_phy_sclk_write;
  wire  system_spiA_phy_data_0_writeEnable;
  reg [1:0] system_spiA_phy_data_0_read;
  wire [1:0] system_spiA_phy_data_0_write;
  wire  system_spiA_phy_data_1_writeEnable;
  reg [1:0] system_spiA_phy_data_1_read;
  wire [1:0] system_spiA_phy_data_1_write;
  wire [0:0] system_spiA_phy_ss;
  reg  _zz_8_;
  reg  _zz_9_;
  reg  _zz_10_;
  reg  sB_IO_3__D_IN_1_regNext;
  reg  _zz_11_;
  reg  sB_IO_4__D_IN_1_regNext;
  wire  system_bridge_cmd_valid;
  wire  system_bridge_cmd_ready;
  wire  system_bridge_cmd_payload_last;
  wire [0:0] system_bridge_cmd_payload_fragment_source;
  wire [0:0] system_bridge_cmd_payload_fragment_opcode;
  wire [31:0] system_bridge_cmd_payload_fragment_address;
  wire [4:0] system_bridge_cmd_payload_fragment_length;
  wire [31:0] system_bridge_cmd_payload_fragment_data;
  wire [3:0] system_bridge_cmd_payload_fragment_mask;
  wire [0:0] system_bridge_cmd_payload_fragment_context;
  wire  system_bridge_rsp_valid;
  wire  system_bridge_rsp_ready;
  wire  system_bridge_rsp_payload_last;
  wire [0:0] system_bridge_rsp_payload_fragment_source;
  wire [0:0] system_bridge_rsp_payload_fragment_opcode;
  wire [31:0] system_bridge_rsp_payload_fragment_data;
  wire [0:0] system_bridge_rsp_payload_fragment_context;
  wire  system_plic_targets_0_gateways_1_source;
  wire  system_plic_targets_0_gateways_2_source;
  wire  generator;
  wire  generator_1_;
  wire  generator_cmd_valid;
  wire  generator_cmd_ready;
  wire  generator_cmd_payload_last;
  wire [0:0] generator_cmd_payload_fragment_opcode;
  wire [23:0] generator_cmd_payload_fragment_address;
  wire [4:0] generator_cmd_payload_fragment_length;
  wire [3:0] generator_cmd_payload_fragment_context;
  wire  generator_rsp_valid;
  wire  generator_rsp_ready;
  wire  generator_rsp_payload_last;
  wire [0:0] generator_rsp_payload_fragment_opcode;
  wire [7:0] generator_rsp_payload_fragment_data;
  wire [3:0] generator_rsp_payload_fragment_context;
  reg [3:0] generator_cmd_payload_fragment_context_regNextWhen;
  wire [1:0] system_plic_targets_0_gateways_1_priority;
  reg  system_plic_targets_0_gateways_1_ip;
  reg  system_plic_targets_0_gateways_1_waitCompletion;
  wire [1:0] system_plic_targets_0_gateways_2_priority;
  reg  system_plic_targets_0_gateways_2_ip;
  reg  system_plic_targets_0_gateways_2_waitCompletion;
  wire [23:0] generator_PADDR;
  wire [0:0] generator_PSEL;
  wire  generator_PENABLE;
  wire  generator_PREADY;
  wire  generator_PWRITE;
  wire [31:0] generator_PWDATA;
  wire [31:0] generator_PRDATA;
  wire  generator_PSLVERROR;
  wire [21:0] system_plic_apb_PADDR;
  wire [0:0] system_plic_apb_PSEL;
  wire  system_plic_apb_PENABLE;
  wire  system_plic_apb_PREADY;
  wire  system_plic_apb_PWRITE;
  wire [31:0] system_plic_apb_PWDATA;
  reg [31:0] system_plic_apb_PRDATA;
  wire  system_plic_apb_PSLVERROR;
  wire  system_plic_bus_askWrite;
  wire  system_plic_bus_askRead;
  wire  system_plic_bus_doWrite;
  wire  system_plic_bus_doRead;
  wire  system_plic_targets_0_ie_0;
  wire  system_plic_targets_0_ie_1;
  wire  system_plic_targets_0_ie_2;
  wire [1:0] system_plic_targets_0_threshold;
  wire [1:0] system_plic_targets_0_requests_0_priority;
  wire [2:0] system_plic_targets_0_requests_0_id;
  wire  system_plic_targets_0_requests_0_valid;
  wire [1:0] system_plic_targets_0_requests_1_priority;
  wire [2:0] system_plic_targets_0_requests_1_id;
  wire  system_plic_targets_0_requests_1_valid;
  wire [1:0] system_plic_targets_0_requests_2_priority;
  wire [2:0] system_plic_targets_0_requests_2_id;
  wire  system_plic_targets_0_requests_2_valid;
  wire [1:0] system_plic_targets_0_requests_3_priority;
  wire [2:0] system_plic_targets_0_requests_3_id;
  wire  system_plic_targets_0_requests_3_valid;
  wire  _zz_12_;
  wire [1:0] _zz_13_;
  wire  _zz_14_;
  wire  _zz_15_;
  wire [1:0] _zz_16_;
  wire  _zz_17_;
  wire  _zz_18_;
  wire [1:0] system_plic_targets_0_bestRequest_priority;
  wire [2:0] system_plic_targets_0_bestRequest_id;
  wire  system_plic_targets_0_bestRequest_valid;
  wire  system_plic_targets_0_iep;
  wire [2:0] system_plic_targets_0_claim;
  reg [1:0] _zz_19_;
  reg [1:0] _zz_20_;
  reg [1:0] _zz_21_;
  reg  system_plic_bridge_claim_valid;
  reg [2:0] system_plic_bridge_claim_payload;
  reg  system_plic_bridge_completion_valid;
  reg [2:0] system_plic_bridge_completion_payload;
  reg [1:0] _zz_22_;
  reg  system_plic_bridge_targetMapping_0_targetCompletion_valid;
  wire [2:0] system_plic_bridge_targetMapping_0_targetCompletion_payload;
  reg  _zz_23_;
  reg  _zz_24_;
  reg  _zz_25_;
  wire  system_bridge_cmd_m2sPipe_valid;
  wire  system_bridge_cmd_m2sPipe_ready;
  wire  system_bridge_cmd_m2sPipe_payload_last;
  wire [0:0] system_bridge_cmd_m2sPipe_payload_fragment_source;
  wire [0:0] system_bridge_cmd_m2sPipe_payload_fragment_opcode;
  wire [31:0] system_bridge_cmd_m2sPipe_payload_fragment_address;
  wire [4:0] system_bridge_cmd_m2sPipe_payload_fragment_length;
  wire [31:0] system_bridge_cmd_m2sPipe_payload_fragment_data;
  wire [3:0] system_bridge_cmd_m2sPipe_payload_fragment_mask;
  wire [0:0] system_bridge_cmd_m2sPipe_payload_fragment_context;
  reg  _zz_26_;
  reg  _zz_27_;
  reg [0:0] _zz_28_;
  reg [0:0] _zz_29_;
  reg [31:0] _zz_30_;
  reg [4:0] _zz_31_;
  reg [31:0] _zz_32_;
  reg [3:0] _zz_33_;
  reg [0:0] _zz_34_;
  assign _zz_81_ = (clockCtrl_holdingLogic_resetCounter != (8'b11111111));
  assign _zz_82_ = system_plic_apb_PWDATA[1 : 1];
  assign _zz_83_ = system_plic_apb_PWDATA[4 : 4];
  assign _zz_84_ = system_plic_apb_PWDATA[5 : 5];
  BufferCC_3_ bufferCC_4_ ( 
    .io_dataIn(clockCtrl_resetRequest),
    .io_dataOut(bufferCC_4__io_dataOut),
    .clk_12M(clk_12M) 
  );
  Apb3UartCtrl system_uartA ( 
    .io_apb_PADDR(_zz_35_),
    .io_apb_PSEL(apb3Router_1__io_outputs_1_PSEL),
    .io_apb_PENABLE(apb3Router_1__io_outputs_1_PENABLE),
    .io_apb_PREADY(system_uartA_io_apb_PREADY),
    .io_apb_PWRITE(apb3Router_1__io_outputs_1_PWRITE),
    .io_apb_PWDATA(apb3Router_1__io_outputs_1_PWDATA),
    .io_apb_PRDATA(system_uartA_io_apb_PRDATA),
    .io_uart_txd(system_uartA_io_uart_txd),
    .io_uart_rxd(system_uartA_uart_rxd),
    .io_interrupt(system_uartA_io_interrupt),
    .clk_12M(clk_12M),
    .clockCtrl_systemReset(clockCtrl_systemReset) 
  );
  Apb3Gpio2 system_gpioA ( 
    .io_gpio_read(_zz_1_),
    .io_gpio_write(system_gpioA_io_gpio_write),
    .io_gpio_writeEnable(system_gpioA_io_gpio_writeEnable),
    .io_bus_PADDR(_zz_36_),
    .io_bus_PSEL(apb3Router_1__io_outputs_2_PSEL),
    .io_bus_PENABLE(apb3Router_1__io_outputs_2_PENABLE),
    .io_bus_PREADY(system_gpioA_io_bus_PREADY),
    .io_bus_PWRITE(apb3Router_1__io_outputs_2_PWRITE),
    .io_bus_PWDATA(apb3Router_1__io_outputs_2_PWDATA),
    .io_bus_PRDATA(system_gpioA_io_bus_PRDATA),
    .io_bus_PSLVERROR(system_gpioA_io_bus_PSLVERROR),
    .io_interrupt(system_gpioA_io_interrupt),
    .clk_12M(clk_12M),
    .clockCtrl_systemReset(clockCtrl_systemReset) 
  );
  MachineTimer system_machineTimer ( 
    .io_bus_PADDR(_zz_37_),
    .io_bus_PSEL(apb3Router_1__io_outputs_4_PSEL),
    .io_bus_PENABLE(apb3Router_1__io_outputs_4_PENABLE),
    .io_bus_PREADY(system_machineTimer_io_bus_PREADY),
    .io_bus_PWRITE(apb3Router_1__io_outputs_4_PWRITE),
    .io_bus_PWDATA(apb3Router_1__io_outputs_4_PWDATA),
    .io_bus_PRDATA(system_machineTimer_io_bus_PRDATA),
    .io_bus_PSLVERROR(system_machineTimer_io_bus_PSLVERROR),
    .io_mTimeInterrupt(system_machineTimer_io_mTimeInterrupt),
    .clk_12M(clk_12M),
    .clockCtrl_systemReset(clockCtrl_systemReset) 
  );
  VexRiscv system_cpu_cpu ( 
    .timerInterrupt(system_machineTimer_io_mTimeInterrupt),
    .externalInterrupt(system_plic_targets_0_iep),
    .softwareInterrupt(_zz_38_),
    .debug_bus_cmd_valid(systemDebugger_1__io_mem_cmd_valid),
    .debug_bus_cmd_ready(system_cpu_cpu_debug_bus_cmd_ready),
    .debug_bus_cmd_payload_wr(systemDebugger_1__io_mem_cmd_payload_wr),
    .debug_bus_cmd_payload_address(_zz_39_),
    .debug_bus_cmd_payload_data(systemDebugger_1__io_mem_cmd_payload_data),
    .debug_bus_rsp_data(system_cpu_cpu_debug_bus_rsp_data),
    .debug_resetOut(system_cpu_cpu_debug_resetOut),
    .iBus_cmd_valid(system_cpu_cpu_iBus_cmd_valid),
    .iBus_cmd_ready(system_cpu_iBus_decoder_io_input_cmd_ready),
    .iBus_cmd_payload_address(system_cpu_cpu_iBus_cmd_payload_address),
    .iBus_cmd_payload_size(system_cpu_cpu_iBus_cmd_payload_size),
    .iBus_rsp_valid(system_cpu_iBus_decoder_io_input_rsp_valid),
    .iBus_rsp_payload_data(system_cpu_iBus_decoder_io_input_rsp_payload_fragment_data),
    .iBus_rsp_payload_error(_zz_40_),
    .dBus_cmd_valid(system_cpu_cpu_dBus_cmd_valid),
    .dBus_cmd_ready(system_cpu_dBus_decoder_io_input_cmd_ready),
    .dBus_cmd_payload_wr(system_cpu_cpu_dBus_cmd_payload_wr),
    .dBus_cmd_payload_address(system_cpu_cpu_dBus_cmd_payload_address),
    .dBus_cmd_payload_data(system_cpu_cpu_dBus_cmd_payload_data),
    .dBus_cmd_payload_size(system_cpu_cpu_dBus_cmd_payload_size),
    .dBus_rsp_ready(_zz_41_),
    .dBus_rsp_error(_zz_42_),
    .dBus_rsp_data(system_cpu_dBus_decoder_io_input_rsp_payload_fragment_data),
    .clk_12M(clk_12M),
    .clockCtrl_systemReset(clockCtrl_systemReset),
    .clockCtrl_resetUnbuffered_regNext(clockCtrl_resetUnbuffered_regNext) 
  );
  JtagBridge jtagBridge_1_ ( 
    .io_jtag_tms(system_cpu_tms),
    .io_jtag_tdi(system_cpu_tdi),
    .io_jtag_tdo(jtagBridge_1__io_jtag_tdo),
    .io_jtag_tck(system_cpu_tck),
    .io_remote_cmd_valid(jtagBridge_1__io_remote_cmd_valid),
    .io_remote_cmd_ready(systemDebugger_1__io_remote_cmd_ready),
    .io_remote_cmd_payload_last(jtagBridge_1__io_remote_cmd_payload_last),
    .io_remote_cmd_payload_fragment(jtagBridge_1__io_remote_cmd_payload_fragment),
    .io_remote_rsp_valid(systemDebugger_1__io_remote_rsp_valid),
    .io_remote_rsp_ready(jtagBridge_1__io_remote_rsp_ready),
    .io_remote_rsp_payload_error(systemDebugger_1__io_remote_rsp_payload_error),
    .io_remote_rsp_payload_data(systemDebugger_1__io_remote_rsp_payload_data),
    .clk_12M(clk_12M),
    .clockCtrl_resetUnbuffered_regNext(clockCtrl_resetUnbuffered_regNext) 
  );
  SystemDebugger systemDebugger_1_ ( 
    .io_remote_cmd_valid(jtagBridge_1__io_remote_cmd_valid),
    .io_remote_cmd_ready(systemDebugger_1__io_remote_cmd_ready),
    .io_remote_cmd_payload_last(jtagBridge_1__io_remote_cmd_payload_last),
    .io_remote_cmd_payload_fragment(jtagBridge_1__io_remote_cmd_payload_fragment),
    .io_remote_rsp_valid(systemDebugger_1__io_remote_rsp_valid),
    .io_remote_rsp_ready(jtagBridge_1__io_remote_rsp_ready),
    .io_remote_rsp_payload_error(systemDebugger_1__io_remote_rsp_payload_error),
    .io_remote_rsp_payload_data(systemDebugger_1__io_remote_rsp_payload_data),
    .io_mem_cmd_valid(systemDebugger_1__io_mem_cmd_valid),
    .io_mem_cmd_ready(system_cpu_cpu_debug_bus_cmd_ready),
    .io_mem_cmd_payload_address(systemDebugger_1__io_mem_cmd_payload_address),
    .io_mem_cmd_payload_data(systemDebugger_1__io_mem_cmd_payload_data),
    .io_mem_cmd_payload_wr(systemDebugger_1__io_mem_cmd_payload_wr),
    .io_mem_cmd_payload_size(systemDebugger_1__io_mem_cmd_payload_size),
    .io_mem_rsp_valid(_zz_7_),
    .io_mem_rsp_payload(system_cpu_cpu_debug_bus_rsp_data),
    .clk_12M(clk_12M),
    .clockCtrl_resetUnbuffered_regNext(clockCtrl_resetUnbuffered_regNext) 
  );
  BmbDecoder system_cpu_iBus_decoder ( 
    .io_input_cmd_valid(system_cpu_cpu_iBus_cmd_valid),
    .io_input_cmd_ready(system_cpu_iBus_decoder_io_input_cmd_ready),
    .io_input_cmd_payload_last(_zz_43_),
    .io_input_cmd_payload_fragment_opcode(_zz_44_),
    .io_input_cmd_payload_fragment_address(system_cpu_cpu_iBus_cmd_payload_address),
    .io_input_cmd_payload_fragment_length(_zz_45_),
    .io_input_rsp_valid(system_cpu_iBus_decoder_io_input_rsp_valid),
    .io_input_rsp_ready(_zz_46_),
    .io_input_rsp_payload_last(system_cpu_iBus_decoder_io_input_rsp_payload_last),
    .io_input_rsp_payload_fragment_opcode(system_cpu_iBus_decoder_io_input_rsp_payload_fragment_opcode),
    .io_input_rsp_payload_fragment_data(system_cpu_iBus_decoder_io_input_rsp_payload_fragment_data),
    .io_outputs_0_cmd_valid(system_cpu_iBus_decoder_io_outputs_0_cmd_valid),
    .io_outputs_0_cmd_ready(system_bridge_bmb_arbiter_io_inputs_1_cmd_ready),
    .io_outputs_0_cmd_payload_last(system_cpu_iBus_decoder_io_outputs_0_cmd_payload_last),
    .io_outputs_0_cmd_payload_fragment_opcode(system_cpu_iBus_decoder_io_outputs_0_cmd_payload_fragment_opcode),
    .io_outputs_0_cmd_payload_fragment_address(system_cpu_iBus_decoder_io_outputs_0_cmd_payload_fragment_address),
    .io_outputs_0_cmd_payload_fragment_length(system_cpu_iBus_decoder_io_outputs_0_cmd_payload_fragment_length),
    .io_outputs_0_rsp_valid(system_bridge_bmb_arbiter_io_inputs_1_rsp_valid),
    .io_outputs_0_rsp_ready(system_cpu_iBus_decoder_io_outputs_0_rsp_ready),
    .io_outputs_0_rsp_payload_last(system_bridge_bmb_arbiter_io_inputs_1_rsp_payload_last),
    .io_outputs_0_rsp_payload_fragment_opcode(system_bridge_bmb_arbiter_io_inputs_1_rsp_payload_fragment_opcode),
    .io_outputs_0_rsp_payload_fragment_data(system_bridge_bmb_arbiter_io_inputs_1_rsp_payload_fragment_data) 
  );
  BmbDecoder_1_ system_cpu_dBus_decoder ( 
    .io_input_cmd_valid(system_cpu_cpu_dBus_cmd_valid),
    .io_input_cmd_ready(system_cpu_dBus_decoder_io_input_cmd_ready),
    .io_input_cmd_payload_last(_zz_47_),
    .io_input_cmd_payload_fragment_opcode(_zz_48_),
    .io_input_cmd_payload_fragment_address(system_cpu_cpu_dBus_cmd_payload_address),
    .io_input_cmd_payload_fragment_length(_zz_5_),
    .io_input_cmd_payload_fragment_data(system_cpu_cpu_dBus_cmd_payload_data),
    .io_input_cmd_payload_fragment_mask(_zz_49_),
    .io_input_cmd_payload_fragment_context(_zz_4_),
    .io_input_rsp_valid(system_cpu_dBus_decoder_io_input_rsp_valid),
    .io_input_rsp_ready(_zz_50_),
    .io_input_rsp_payload_last(system_cpu_dBus_decoder_io_input_rsp_payload_last),
    .io_input_rsp_payload_fragment_opcode(system_cpu_dBus_decoder_io_input_rsp_payload_fragment_opcode),
    .io_input_rsp_payload_fragment_data(system_cpu_dBus_decoder_io_input_rsp_payload_fragment_data),
    .io_input_rsp_payload_fragment_context(system_cpu_dBus_decoder_io_input_rsp_payload_fragment_context),
    .io_outputs_0_cmd_valid(system_cpu_dBus_decoder_io_outputs_0_cmd_valid),
    .io_outputs_0_cmd_ready(system_bridge_bmb_arbiter_io_inputs_0_cmd_ready),
    .io_outputs_0_cmd_payload_last(system_cpu_dBus_decoder_io_outputs_0_cmd_payload_last),
    .io_outputs_0_cmd_payload_fragment_opcode(system_cpu_dBus_decoder_io_outputs_0_cmd_payload_fragment_opcode),
    .io_outputs_0_cmd_payload_fragment_address(system_cpu_dBus_decoder_io_outputs_0_cmd_payload_fragment_address),
    .io_outputs_0_cmd_payload_fragment_length(system_cpu_dBus_decoder_io_outputs_0_cmd_payload_fragment_length),
    .io_outputs_0_cmd_payload_fragment_data(system_cpu_dBus_decoder_io_outputs_0_cmd_payload_fragment_data),
    .io_outputs_0_cmd_payload_fragment_mask(system_cpu_dBus_decoder_io_outputs_0_cmd_payload_fragment_mask),
    .io_outputs_0_cmd_payload_fragment_context(system_cpu_dBus_decoder_io_outputs_0_cmd_payload_fragment_context),
    .io_outputs_0_rsp_valid(system_bridge_bmb_arbiter_io_inputs_0_rsp_valid),
    .io_outputs_0_rsp_ready(system_cpu_dBus_decoder_io_outputs_0_rsp_ready),
    .io_outputs_0_rsp_payload_last(system_bridge_bmb_arbiter_io_inputs_0_rsp_payload_last),
    .io_outputs_0_rsp_payload_fragment_opcode(system_bridge_bmb_arbiter_io_inputs_0_rsp_payload_fragment_opcode),
    .io_outputs_0_rsp_payload_fragment_data(system_bridge_bmb_arbiter_io_inputs_0_rsp_payload_fragment_data),
    .io_outputs_0_rsp_payload_fragment_context(system_bridge_bmb_arbiter_io_inputs_0_rsp_payload_fragment_context) 
  );
  Apb3SpiXdrMasterCtrl system_spiA ( 
    .io_apb_PADDR(_zz_51_),
    .io_apb_PSEL(apb3Router_1__io_outputs_3_PSEL),
    .io_apb_PENABLE(apb3Router_1__io_outputs_3_PENABLE),
    .io_apb_PREADY(system_spiA_io_apb_PREADY),
    .io_apb_PWRITE(apb3Router_1__io_outputs_3_PWRITE),
    .io_apb_PWDATA(apb3Router_1__io_outputs_3_PWDATA),
    .io_apb_PRDATA(system_spiA_io_apb_PRDATA),
    .io_xip_cmd_valid(generator_cmd_valid),
    .io_xip_cmd_ready(system_spiA_io_xip_cmd_ready),
    .io_xip_cmd_payload_address(generator_cmd_payload_fragment_address),
    .io_xip_cmd_payload_length(generator_cmd_payload_fragment_length),
    .io_xip_rsp_valid(system_spiA_io_xip_rsp_valid),
    .io_xip_rsp_ready(generator_rsp_ready),
    .io_xip_rsp_payload_last(system_spiA_io_xip_rsp_payload_last),
    .io_xip_rsp_payload_fragment(system_spiA_io_xip_rsp_payload_fragment),
    .io_spi_sclk_write(system_spiA_io_spi_sclk_write),
    .io_spi_data_0_writeEnable(system_spiA_io_spi_data_0_writeEnable),
    .io_spi_data_0_read(system_spiA_phy_data_0_read),
    .io_spi_data_0_write(system_spiA_io_spi_data_0_write),
    .io_spi_data_1_writeEnable(system_spiA_io_spi_data_1_writeEnable),
    .io_spi_data_1_read(system_spiA_phy_data_1_read),
    .io_spi_data_1_write(system_spiA_io_spi_data_1_write),
    .io_spi_ss(system_spiA_io_spi_ss),
    .io_interrupt(system_spiA_io_interrupt),
    .clk_12M(clk_12M),
    .clockCtrl_systemReset(clockCtrl_systemReset) 
  );
  SB_IO #( 
    .PIN_TYPE((6'b010000)) 
  ) sB_IO_1_ ( 
    .PACKAGE_PIN(system_spiA_spi_ss_0),
    .CLOCK_ENABLE(_zz_52_),
    .INPUT_CLK(_zz_53_),
    .OUTPUT_CLK(clk_12M),
    .OUTPUT_ENABLE(_zz_54_),
    .D_OUT_0(_zz_55_),
    .D_OUT_1(_zz_8_),
    .D_IN_0(sB_IO_1__D_IN_0),
    .D_IN_1(sB_IO_1__D_IN_1) 
  );
  SB_IO #( 
    .PIN_TYPE((6'b010000)) 
  ) sB_IO_2_ ( 
    .PACKAGE_PIN(system_spiA_spi_sclk),
    .CLOCK_ENABLE(_zz_56_),
    .INPUT_CLK(_zz_57_),
    .OUTPUT_CLK(clk_12M),
    .OUTPUT_ENABLE(_zz_58_),
    .D_OUT_0(_zz_59_),
    .D_OUT_1(_zz_9_),
    .D_IN_0(sB_IO_2__D_IN_0),
    .D_IN_1(sB_IO_2__D_IN_1) 
  );
  SB_IO #( 
    .PIN_TYPE((6'b110000)) 
  ) sB_IO_3_ ( 
    .PACKAGE_PIN(system_spiA_spi_data_0),
    .CLOCK_ENABLE(_zz_60_),
    .INPUT_CLK(clk_12M),
    .OUTPUT_CLK(clk_12M),
    .OUTPUT_ENABLE(system_spiA_phy_data_0_writeEnable),
    .D_OUT_0(_zz_61_),
    .D_OUT_1(_zz_10_),
    .D_IN_0(sB_IO_3__D_IN_0),
    .D_IN_1(sB_IO_3__D_IN_1) 
  );
  SB_IO #( 
    .PIN_TYPE((6'b110000)) 
  ) sB_IO_4_ ( 
    .PACKAGE_PIN(system_spiA_spi_data_1),
    .CLOCK_ENABLE(_zz_62_),
    .INPUT_CLK(clk_12M),
    .OUTPUT_CLK(clk_12M),
    .OUTPUT_ENABLE(system_spiA_phy_data_1_writeEnable),
    .D_OUT_0(_zz_63_),
    .D_OUT_1(_zz_11_),
    .D_IN_0(sB_IO_4__D_IN_0),
    .D_IN_1(sB_IO_4__D_IN_1) 
  );
  BmbArbiter system_bridge_bmb_arbiter ( 
    .io_inputs_0_cmd_valid(system_cpu_dBus_decoder_io_outputs_0_cmd_valid),
    .io_inputs_0_cmd_ready(system_bridge_bmb_arbiter_io_inputs_0_cmd_ready),
    .io_inputs_0_cmd_payload_last(system_cpu_dBus_decoder_io_outputs_0_cmd_payload_last),
    .io_inputs_0_cmd_payload_fragment_opcode(system_cpu_dBus_decoder_io_outputs_0_cmd_payload_fragment_opcode),
    .io_inputs_0_cmd_payload_fragment_address(system_cpu_dBus_decoder_io_outputs_0_cmd_payload_fragment_address),
    .io_inputs_0_cmd_payload_fragment_length(_zz_64_),
    .io_inputs_0_cmd_payload_fragment_data(system_cpu_dBus_decoder_io_outputs_0_cmd_payload_fragment_data),
    .io_inputs_0_cmd_payload_fragment_mask(system_cpu_dBus_decoder_io_outputs_0_cmd_payload_fragment_mask),
    .io_inputs_0_cmd_payload_fragment_context(system_cpu_dBus_decoder_io_outputs_0_cmd_payload_fragment_context),
    .io_inputs_0_rsp_valid(system_bridge_bmb_arbiter_io_inputs_0_rsp_valid),
    .io_inputs_0_rsp_ready(system_cpu_dBus_decoder_io_outputs_0_rsp_ready),
    .io_inputs_0_rsp_payload_last(system_bridge_bmb_arbiter_io_inputs_0_rsp_payload_last),
    .io_inputs_0_rsp_payload_fragment_opcode(system_bridge_bmb_arbiter_io_inputs_0_rsp_payload_fragment_opcode),
    .io_inputs_0_rsp_payload_fragment_data(system_bridge_bmb_arbiter_io_inputs_0_rsp_payload_fragment_data),
    .io_inputs_0_rsp_payload_fragment_context(system_bridge_bmb_arbiter_io_inputs_0_rsp_payload_fragment_context),
    .io_inputs_1_cmd_valid(system_cpu_iBus_decoder_io_outputs_0_cmd_valid),
    .io_inputs_1_cmd_ready(system_bridge_bmb_arbiter_io_inputs_1_cmd_ready),
    .io_inputs_1_cmd_payload_last(system_cpu_iBus_decoder_io_outputs_0_cmd_payload_last),
    .io_inputs_1_cmd_payload_fragment_opcode(system_cpu_iBus_decoder_io_outputs_0_cmd_payload_fragment_opcode),
    .io_inputs_1_cmd_payload_fragment_address(system_cpu_iBus_decoder_io_outputs_0_cmd_payload_fragment_address),
    .io_inputs_1_cmd_payload_fragment_length(system_cpu_iBus_decoder_io_outputs_0_cmd_payload_fragment_length),
    .io_inputs_1_cmd_payload_fragment_data(_zz_65_),
    .io_inputs_1_cmd_payload_fragment_mask(_zz_66_),
    .io_inputs_1_cmd_payload_fragment_context(_zz_67_),
    .io_inputs_1_rsp_valid(system_bridge_bmb_arbiter_io_inputs_1_rsp_valid),
    .io_inputs_1_rsp_ready(system_cpu_iBus_decoder_io_outputs_0_rsp_ready),
    .io_inputs_1_rsp_payload_last(system_bridge_bmb_arbiter_io_inputs_1_rsp_payload_last),
    .io_inputs_1_rsp_payload_fragment_opcode(system_bridge_bmb_arbiter_io_inputs_1_rsp_payload_fragment_opcode),
    .io_inputs_1_rsp_payload_fragment_data(system_bridge_bmb_arbiter_io_inputs_1_rsp_payload_fragment_data),
    .io_inputs_1_rsp_payload_fragment_context(system_bridge_bmb_arbiter_io_inputs_1_rsp_payload_fragment_context),
    .io_output_cmd_valid(system_bridge_bmb_arbiter_io_output_cmd_valid),
    .io_output_cmd_ready(system_bridge_cmd_ready),
    .io_output_cmd_payload_last(system_bridge_bmb_arbiter_io_output_cmd_payload_last),
    .io_output_cmd_payload_fragment_source(system_bridge_bmb_arbiter_io_output_cmd_payload_fragment_source),
    .io_output_cmd_payload_fragment_opcode(system_bridge_bmb_arbiter_io_output_cmd_payload_fragment_opcode),
    .io_output_cmd_payload_fragment_address(system_bridge_bmb_arbiter_io_output_cmd_payload_fragment_address),
    .io_output_cmd_payload_fragment_length(system_bridge_bmb_arbiter_io_output_cmd_payload_fragment_length),
    .io_output_cmd_payload_fragment_data(system_bridge_bmb_arbiter_io_output_cmd_payload_fragment_data),
    .io_output_cmd_payload_fragment_mask(system_bridge_bmb_arbiter_io_output_cmd_payload_fragment_mask),
    .io_output_cmd_payload_fragment_context(system_bridge_bmb_arbiter_io_output_cmd_payload_fragment_context),
    .io_output_rsp_valid(system_bridge_rsp_valid),
    .io_output_rsp_ready(system_bridge_bmb_arbiter_io_output_rsp_ready),
    .io_output_rsp_payload_last(system_bridge_rsp_payload_last),
    .io_output_rsp_payload_fragment_source(system_bridge_rsp_payload_fragment_source),
    .io_output_rsp_payload_fragment_opcode(system_bridge_rsp_payload_fragment_opcode),
    .io_output_rsp_payload_fragment_data(system_bridge_rsp_payload_fragment_data),
    .io_output_rsp_payload_fragment_context(system_bridge_rsp_payload_fragment_context),
    .clk_12M(clk_12M),
    .clockCtrl_systemReset(clockCtrl_systemReset) 
  );
  BmbIce40Spram system_ramA ( 
    .io_bus_cmd_valid(system_ramA_bmb_burstUnburstifier_io_output_cmd_valid),
    .io_bus_cmd_ready(system_ramA_io_bus_cmd_ready),
    .io_bus_cmd_payload_last(system_ramA_bmb_burstUnburstifier_io_output_cmd_payload_last),
    .io_bus_cmd_payload_fragment_source(system_ramA_bmb_burstUnburstifier_io_output_cmd_payload_fragment_source),
    .io_bus_cmd_payload_fragment_opcode(system_ramA_bmb_burstUnburstifier_io_output_cmd_payload_fragment_opcode),
    .io_bus_cmd_payload_fragment_address(system_ramA_bmb_burstUnburstifier_io_output_cmd_payload_fragment_address),
    .io_bus_cmd_payload_fragment_length(system_ramA_bmb_burstUnburstifier_io_output_cmd_payload_fragment_length),
    .io_bus_cmd_payload_fragment_data(system_ramA_bmb_burstUnburstifier_io_output_cmd_payload_fragment_data),
    .io_bus_cmd_payload_fragment_mask(system_ramA_bmb_burstUnburstifier_io_output_cmd_payload_fragment_mask),
    .io_bus_cmd_payload_fragment_context(system_ramA_bmb_burstUnburstifier_io_output_cmd_payload_fragment_context),
    .io_bus_rsp_valid(system_ramA_io_bus_rsp_valid),
    .io_bus_rsp_ready(system_ramA_bmb_burstUnburstifier_io_output_rsp_ready),
    .io_bus_rsp_payload_last(system_ramA_io_bus_rsp_payload_last),
    .io_bus_rsp_payload_fragment_source(system_ramA_io_bus_rsp_payload_fragment_source),
    .io_bus_rsp_payload_fragment_opcode(system_ramA_io_bus_rsp_payload_fragment_opcode),
    .io_bus_rsp_payload_fragment_data(system_ramA_io_bus_rsp_payload_fragment_data),
    .io_bus_rsp_payload_fragment_context(system_ramA_io_bus_rsp_payload_fragment_context),
    .clk_12M(clk_12M),
    .clockCtrl_systemReset(clockCtrl_systemReset) 
  );
  BmbArbiter_1_ system_ramA_bmb_arbiter ( 
    .io_inputs_0_cmd_valid(system_bridge_bmb_decoder_io_outputs_0_cmd_valid),
    .io_inputs_0_cmd_ready(system_ramA_bmb_arbiter_io_inputs_0_cmd_ready),
    .io_inputs_0_cmd_payload_last(system_bridge_bmb_decoder_io_outputs_0_cmd_payload_last),
    .io_inputs_0_cmd_payload_fragment_source(system_bridge_bmb_decoder_io_outputs_0_cmd_payload_fragment_source),
    .io_inputs_0_cmd_payload_fragment_opcode(system_bridge_bmb_decoder_io_outputs_0_cmd_payload_fragment_opcode),
    .io_inputs_0_cmd_payload_fragment_address(_zz_68_),
    .io_inputs_0_cmd_payload_fragment_length(system_bridge_bmb_decoder_io_outputs_0_cmd_payload_fragment_length),
    .io_inputs_0_cmd_payload_fragment_data(system_bridge_bmb_decoder_io_outputs_0_cmd_payload_fragment_data),
    .io_inputs_0_cmd_payload_fragment_mask(system_bridge_bmb_decoder_io_outputs_0_cmd_payload_fragment_mask),
    .io_inputs_0_cmd_payload_fragment_context(system_bridge_bmb_decoder_io_outputs_0_cmd_payload_fragment_context),
    .io_inputs_0_rsp_valid(system_ramA_bmb_arbiter_io_inputs_0_rsp_valid),
    .io_inputs_0_rsp_ready(system_bridge_bmb_decoder_io_outputs_0_rsp_ready),
    .io_inputs_0_rsp_payload_last(system_ramA_bmb_arbiter_io_inputs_0_rsp_payload_last),
    .io_inputs_0_rsp_payload_fragment_source(system_ramA_bmb_arbiter_io_inputs_0_rsp_payload_fragment_source),
    .io_inputs_0_rsp_payload_fragment_opcode(system_ramA_bmb_arbiter_io_inputs_0_rsp_payload_fragment_opcode),
    .io_inputs_0_rsp_payload_fragment_data(system_ramA_bmb_arbiter_io_inputs_0_rsp_payload_fragment_data),
    .io_inputs_0_rsp_payload_fragment_context(system_ramA_bmb_arbiter_io_inputs_0_rsp_payload_fragment_context),
    .io_output_cmd_valid(system_ramA_bmb_arbiter_io_output_cmd_valid),
    .io_output_cmd_ready(system_ramA_bmb_burstUnburstifier_io_input_cmd_ready),
    .io_output_cmd_payload_last(system_ramA_bmb_arbiter_io_output_cmd_payload_last),
    .io_output_cmd_payload_fragment_source(system_ramA_bmb_arbiter_io_output_cmd_payload_fragment_source),
    .io_output_cmd_payload_fragment_opcode(system_ramA_bmb_arbiter_io_output_cmd_payload_fragment_opcode),
    .io_output_cmd_payload_fragment_address(system_ramA_bmb_arbiter_io_output_cmd_payload_fragment_address),
    .io_output_cmd_payload_fragment_length(system_ramA_bmb_arbiter_io_output_cmd_payload_fragment_length),
    .io_output_cmd_payload_fragment_data(system_ramA_bmb_arbiter_io_output_cmd_payload_fragment_data),
    .io_output_cmd_payload_fragment_mask(system_ramA_bmb_arbiter_io_output_cmd_payload_fragment_mask),
    .io_output_cmd_payload_fragment_context(system_ramA_bmb_arbiter_io_output_cmd_payload_fragment_context),
    .io_output_rsp_valid(system_ramA_bmb_burstUnburstifier_io_input_rsp_valid),
    .io_output_rsp_ready(system_ramA_bmb_arbiter_io_output_rsp_ready),
    .io_output_rsp_payload_last(system_ramA_bmb_burstUnburstifier_io_input_rsp_payload_last),
    .io_output_rsp_payload_fragment_source(system_ramA_bmb_burstUnburstifier_io_input_rsp_payload_fragment_source),
    .io_output_rsp_payload_fragment_opcode(system_ramA_bmb_burstUnburstifier_io_input_rsp_payload_fragment_opcode),
    .io_output_rsp_payload_fragment_data(system_ramA_bmb_burstUnburstifier_io_input_rsp_payload_fragment_data),
    .io_output_rsp_payload_fragment_context(system_ramA_bmb_burstUnburstifier_io_input_rsp_payload_fragment_context) 
  );
  BmbUnburstify system_ramA_bmb_burstUnburstifier ( 
    .io_input_cmd_valid(system_ramA_bmb_arbiter_io_output_cmd_valid),
    .io_input_cmd_ready(system_ramA_bmb_burstUnburstifier_io_input_cmd_ready),
    .io_input_cmd_payload_last(system_ramA_bmb_arbiter_io_output_cmd_payload_last),
    .io_input_cmd_payload_fragment_source(system_ramA_bmb_arbiter_io_output_cmd_payload_fragment_source),
    .io_input_cmd_payload_fragment_opcode(system_ramA_bmb_arbiter_io_output_cmd_payload_fragment_opcode),
    .io_input_cmd_payload_fragment_address(system_ramA_bmb_arbiter_io_output_cmd_payload_fragment_address),
    .io_input_cmd_payload_fragment_length(system_ramA_bmb_arbiter_io_output_cmd_payload_fragment_length),
    .io_input_cmd_payload_fragment_data(system_ramA_bmb_arbiter_io_output_cmd_payload_fragment_data),
    .io_input_cmd_payload_fragment_mask(system_ramA_bmb_arbiter_io_output_cmd_payload_fragment_mask),
    .io_input_cmd_payload_fragment_context(system_ramA_bmb_arbiter_io_output_cmd_payload_fragment_context),
    .io_input_rsp_valid(system_ramA_bmb_burstUnburstifier_io_input_rsp_valid),
    .io_input_rsp_ready(system_ramA_bmb_arbiter_io_output_rsp_ready),
    .io_input_rsp_payload_last(system_ramA_bmb_burstUnburstifier_io_input_rsp_payload_last),
    .io_input_rsp_payload_fragment_source(system_ramA_bmb_burstUnburstifier_io_input_rsp_payload_fragment_source),
    .io_input_rsp_payload_fragment_opcode(system_ramA_bmb_burstUnburstifier_io_input_rsp_payload_fragment_opcode),
    .io_input_rsp_payload_fragment_data(system_ramA_bmb_burstUnburstifier_io_input_rsp_payload_fragment_data),
    .io_input_rsp_payload_fragment_context(system_ramA_bmb_burstUnburstifier_io_input_rsp_payload_fragment_context),
    .io_output_cmd_valid(system_ramA_bmb_burstUnburstifier_io_output_cmd_valid),
    .io_output_cmd_ready(system_ramA_io_bus_cmd_ready),
    .io_output_cmd_payload_last(system_ramA_bmb_burstUnburstifier_io_output_cmd_payload_last),
    .io_output_cmd_payload_fragment_source(system_ramA_bmb_burstUnburstifier_io_output_cmd_payload_fragment_source),
    .io_output_cmd_payload_fragment_opcode(system_ramA_bmb_burstUnburstifier_io_output_cmd_payload_fragment_opcode),
    .io_output_cmd_payload_fragment_address(system_ramA_bmb_burstUnburstifier_io_output_cmd_payload_fragment_address),
    .io_output_cmd_payload_fragment_length(system_ramA_bmb_burstUnburstifier_io_output_cmd_payload_fragment_length),
    .io_output_cmd_payload_fragment_data(system_ramA_bmb_burstUnburstifier_io_output_cmd_payload_fragment_data),
    .io_output_cmd_payload_fragment_mask(system_ramA_bmb_burstUnburstifier_io_output_cmd_payload_fragment_mask),
    .io_output_cmd_payload_fragment_context(system_ramA_bmb_burstUnburstifier_io_output_cmd_payload_fragment_context),
    .io_output_rsp_valid(system_ramA_io_bus_rsp_valid),
    .io_output_rsp_ready(system_ramA_bmb_burstUnburstifier_io_output_rsp_ready),
    .io_output_rsp_payload_last(system_ramA_io_bus_rsp_payload_last),
    .io_output_rsp_payload_fragment_source(system_ramA_io_bus_rsp_payload_fragment_source),
    .io_output_rsp_payload_fragment_opcode(system_ramA_io_bus_rsp_payload_fragment_opcode),
    .io_output_rsp_payload_fragment_data(system_ramA_io_bus_rsp_payload_fragment_data),
    .io_output_rsp_payload_fragment_context(system_ramA_io_bus_rsp_payload_fragment_context),
    .clk_12M(clk_12M),
    .clockCtrl_systemReset(clockCtrl_systemReset) 
  );
  BmbDecoder_2_ system_bridge_bmb_decoder ( 
    .io_input_cmd_valid(system_bridge_cmd_m2sPipe_valid),
    .io_input_cmd_ready(system_bridge_bmb_decoder_io_input_cmd_ready),
    .io_input_cmd_payload_last(system_bridge_cmd_m2sPipe_payload_last),
    .io_input_cmd_payload_fragment_source(system_bridge_cmd_m2sPipe_payload_fragment_source),
    .io_input_cmd_payload_fragment_opcode(system_bridge_cmd_m2sPipe_payload_fragment_opcode),
    .io_input_cmd_payload_fragment_address(system_bridge_cmd_m2sPipe_payload_fragment_address),
    .io_input_cmd_payload_fragment_length(system_bridge_cmd_m2sPipe_payload_fragment_length),
    .io_input_cmd_payload_fragment_data(system_bridge_cmd_m2sPipe_payload_fragment_data),
    .io_input_cmd_payload_fragment_mask(system_bridge_cmd_m2sPipe_payload_fragment_mask),
    .io_input_cmd_payload_fragment_context(system_bridge_cmd_m2sPipe_payload_fragment_context),
    .io_input_rsp_valid(system_bridge_bmb_decoder_io_input_rsp_valid),
    .io_input_rsp_ready(system_bridge_rsp_ready),
    .io_input_rsp_payload_last(system_bridge_bmb_decoder_io_input_rsp_payload_last),
    .io_input_rsp_payload_fragment_source(system_bridge_bmb_decoder_io_input_rsp_payload_fragment_source),
    .io_input_rsp_payload_fragment_opcode(system_bridge_bmb_decoder_io_input_rsp_payload_fragment_opcode),
    .io_input_rsp_payload_fragment_data(system_bridge_bmb_decoder_io_input_rsp_payload_fragment_data),
    .io_input_rsp_payload_fragment_context(system_bridge_bmb_decoder_io_input_rsp_payload_fragment_context),
    .io_outputs_0_cmd_valid(system_bridge_bmb_decoder_io_outputs_0_cmd_valid),
    .io_outputs_0_cmd_ready(system_ramA_bmb_arbiter_io_inputs_0_cmd_ready),
    .io_outputs_0_cmd_payload_last(system_bridge_bmb_decoder_io_outputs_0_cmd_payload_last),
    .io_outputs_0_cmd_payload_fragment_source(system_bridge_bmb_decoder_io_outputs_0_cmd_payload_fragment_source),
    .io_outputs_0_cmd_payload_fragment_opcode(system_bridge_bmb_decoder_io_outputs_0_cmd_payload_fragment_opcode),
    .io_outputs_0_cmd_payload_fragment_address(system_bridge_bmb_decoder_io_outputs_0_cmd_payload_fragment_address),
    .io_outputs_0_cmd_payload_fragment_length(system_bridge_bmb_decoder_io_outputs_0_cmd_payload_fragment_length),
    .io_outputs_0_cmd_payload_fragment_data(system_bridge_bmb_decoder_io_outputs_0_cmd_payload_fragment_data),
    .io_outputs_0_cmd_payload_fragment_mask(system_bridge_bmb_decoder_io_outputs_0_cmd_payload_fragment_mask),
    .io_outputs_0_cmd_payload_fragment_context(system_bridge_bmb_decoder_io_outputs_0_cmd_payload_fragment_context),
    .io_outputs_0_rsp_valid(system_ramA_bmb_arbiter_io_inputs_0_rsp_valid),
    .io_outputs_0_rsp_ready(system_bridge_bmb_decoder_io_outputs_0_rsp_ready),
    .io_outputs_0_rsp_payload_last(system_ramA_bmb_arbiter_io_inputs_0_rsp_payload_last),
    .io_outputs_0_rsp_payload_fragment_source(system_ramA_bmb_arbiter_io_inputs_0_rsp_payload_fragment_source),
    .io_outputs_0_rsp_payload_fragment_opcode(system_ramA_bmb_arbiter_io_inputs_0_rsp_payload_fragment_opcode),
    .io_outputs_0_rsp_payload_fragment_data(system_ramA_bmb_arbiter_io_inputs_0_rsp_payload_fragment_data),
    .io_outputs_0_rsp_payload_fragment_context(system_ramA_bmb_arbiter_io_inputs_0_rsp_payload_fragment_context),
    .io_outputs_1_cmd_valid(system_bridge_bmb_decoder_io_outputs_1_cmd_valid),
    .io_outputs_1_cmd_ready(system_peripheralBridge_input_arbiter_io_inputs_0_cmd_ready),
    .io_outputs_1_cmd_payload_last(system_bridge_bmb_decoder_io_outputs_1_cmd_payload_last),
    .io_outputs_1_cmd_payload_fragment_source(system_bridge_bmb_decoder_io_outputs_1_cmd_payload_fragment_source),
    .io_outputs_1_cmd_payload_fragment_opcode(system_bridge_bmb_decoder_io_outputs_1_cmd_payload_fragment_opcode),
    .io_outputs_1_cmd_payload_fragment_address(system_bridge_bmb_decoder_io_outputs_1_cmd_payload_fragment_address),
    .io_outputs_1_cmd_payload_fragment_length(system_bridge_bmb_decoder_io_outputs_1_cmd_payload_fragment_length),
    .io_outputs_1_cmd_payload_fragment_data(system_bridge_bmb_decoder_io_outputs_1_cmd_payload_fragment_data),
    .io_outputs_1_cmd_payload_fragment_mask(system_bridge_bmb_decoder_io_outputs_1_cmd_payload_fragment_mask),
    .io_outputs_1_cmd_payload_fragment_context(system_bridge_bmb_decoder_io_outputs_1_cmd_payload_fragment_context),
    .io_outputs_1_rsp_valid(system_peripheralBridge_input_arbiter_io_inputs_0_rsp_valid),
    .io_outputs_1_rsp_ready(system_bridge_bmb_decoder_io_outputs_1_rsp_ready),
    .io_outputs_1_rsp_payload_last(system_peripheralBridge_input_arbiter_io_inputs_0_rsp_payload_last),
    .io_outputs_1_rsp_payload_fragment_source(system_peripheralBridge_input_arbiter_io_inputs_0_rsp_payload_fragment_source),
    .io_outputs_1_rsp_payload_fragment_opcode(system_peripheralBridge_input_arbiter_io_inputs_0_rsp_payload_fragment_opcode),
    .io_outputs_1_rsp_payload_fragment_data(system_peripheralBridge_input_arbiter_io_inputs_0_rsp_payload_fragment_data),
    .io_outputs_1_rsp_payload_fragment_context(system_peripheralBridge_input_arbiter_io_inputs_0_rsp_payload_fragment_context),
    .io_outputs_2_cmd_valid(system_bridge_bmb_decoder_io_outputs_2_cmd_valid),
    .io_outputs_2_cmd_ready(system_spiA_bmb_arbiter_io_inputs_0_cmd_ready),
    .io_outputs_2_cmd_payload_last(system_bridge_bmb_decoder_io_outputs_2_cmd_payload_last),
    .io_outputs_2_cmd_payload_fragment_source(system_bridge_bmb_decoder_io_outputs_2_cmd_payload_fragment_source),
    .io_outputs_2_cmd_payload_fragment_opcode(system_bridge_bmb_decoder_io_outputs_2_cmd_payload_fragment_opcode),
    .io_outputs_2_cmd_payload_fragment_address(system_bridge_bmb_decoder_io_outputs_2_cmd_payload_fragment_address),
    .io_outputs_2_cmd_payload_fragment_length(system_bridge_bmb_decoder_io_outputs_2_cmd_payload_fragment_length),
    .io_outputs_2_cmd_payload_fragment_data(system_bridge_bmb_decoder_io_outputs_2_cmd_payload_fragment_data),
    .io_outputs_2_cmd_payload_fragment_mask(system_bridge_bmb_decoder_io_outputs_2_cmd_payload_fragment_mask),
    .io_outputs_2_cmd_payload_fragment_context(system_bridge_bmb_decoder_io_outputs_2_cmd_payload_fragment_context),
    .io_outputs_2_rsp_valid(system_spiA_bmb_arbiter_io_inputs_0_rsp_valid),
    .io_outputs_2_rsp_ready(system_bridge_bmb_decoder_io_outputs_2_rsp_ready),
    .io_outputs_2_rsp_payload_last(system_spiA_bmb_arbiter_io_inputs_0_rsp_payload_last),
    .io_outputs_2_rsp_payload_fragment_source(system_spiA_bmb_arbiter_io_inputs_0_rsp_payload_fragment_source),
    .io_outputs_2_rsp_payload_fragment_opcode(system_spiA_bmb_arbiter_io_inputs_0_rsp_payload_fragment_opcode),
    .io_outputs_2_rsp_payload_fragment_data(system_spiA_bmb_arbiter_io_inputs_0_rsp_payload_fragment_data),
    .io_outputs_2_rsp_payload_fragment_context(system_spiA_bmb_arbiter_io_inputs_0_rsp_payload_fragment_context),
    .clk_12M(clk_12M),
    .clockCtrl_systemReset(clockCtrl_systemReset) 
  );
  BmbArbiter_2_ system_spiA_bmb_arbiter ( 
    .io_inputs_0_cmd_valid(system_bridge_bmb_decoder_io_outputs_2_cmd_valid),
    .io_inputs_0_cmd_ready(system_spiA_bmb_arbiter_io_inputs_0_cmd_ready),
    .io_inputs_0_cmd_payload_last(system_bridge_bmb_decoder_io_outputs_2_cmd_payload_last),
    .io_inputs_0_cmd_payload_fragment_source(system_bridge_bmb_decoder_io_outputs_2_cmd_payload_fragment_source),
    .io_inputs_0_cmd_payload_fragment_opcode(system_bridge_bmb_decoder_io_outputs_2_cmd_payload_fragment_opcode),
    .io_inputs_0_cmd_payload_fragment_address(_zz_69_),
    .io_inputs_0_cmd_payload_fragment_length(system_bridge_bmb_decoder_io_outputs_2_cmd_payload_fragment_length),
    .io_inputs_0_cmd_payload_fragment_context(system_bridge_bmb_decoder_io_outputs_2_cmd_payload_fragment_context),
    .io_inputs_0_rsp_valid(system_spiA_bmb_arbiter_io_inputs_0_rsp_valid),
    .io_inputs_0_rsp_ready(system_bridge_bmb_decoder_io_outputs_2_rsp_ready),
    .io_inputs_0_rsp_payload_last(system_spiA_bmb_arbiter_io_inputs_0_rsp_payload_last),
    .io_inputs_0_rsp_payload_fragment_source(system_spiA_bmb_arbiter_io_inputs_0_rsp_payload_fragment_source),
    .io_inputs_0_rsp_payload_fragment_opcode(system_spiA_bmb_arbiter_io_inputs_0_rsp_payload_fragment_opcode),
    .io_inputs_0_rsp_payload_fragment_data(system_spiA_bmb_arbiter_io_inputs_0_rsp_payload_fragment_data),
    .io_inputs_0_rsp_payload_fragment_context(system_spiA_bmb_arbiter_io_inputs_0_rsp_payload_fragment_context),
    .io_output_cmd_valid(system_spiA_bmb_arbiter_io_output_cmd_valid),
    .io_output_cmd_ready(system_spiA_bmb_downSizer_io_input_cmd_ready),
    .io_output_cmd_payload_last(system_spiA_bmb_arbiter_io_output_cmd_payload_last),
    .io_output_cmd_payload_fragment_source(system_spiA_bmb_arbiter_io_output_cmd_payload_fragment_source),
    .io_output_cmd_payload_fragment_opcode(system_spiA_bmb_arbiter_io_output_cmd_payload_fragment_opcode),
    .io_output_cmd_payload_fragment_address(system_spiA_bmb_arbiter_io_output_cmd_payload_fragment_address),
    .io_output_cmd_payload_fragment_length(system_spiA_bmb_arbiter_io_output_cmd_payload_fragment_length),
    .io_output_cmd_payload_fragment_context(system_spiA_bmb_arbiter_io_output_cmd_payload_fragment_context),
    .io_output_rsp_valid(system_spiA_bmb_downSizer_io_input_rsp_valid),
    .io_output_rsp_ready(system_spiA_bmb_arbiter_io_output_rsp_ready),
    .io_output_rsp_payload_last(system_spiA_bmb_downSizer_io_input_rsp_payload_last),
    .io_output_rsp_payload_fragment_source(system_spiA_bmb_downSizer_io_input_rsp_payload_fragment_source),
    .io_output_rsp_payload_fragment_opcode(system_spiA_bmb_downSizer_io_input_rsp_payload_fragment_opcode),
    .io_output_rsp_payload_fragment_data(system_spiA_bmb_downSizer_io_input_rsp_payload_fragment_data),
    .io_output_rsp_payload_fragment_context(system_spiA_bmb_downSizer_io_input_rsp_payload_fragment_context) 
  );
  BmbDownSizerBridge system_spiA_bmb_downSizer ( 
    .io_input_cmd_valid(system_spiA_bmb_arbiter_io_output_cmd_valid),
    .io_input_cmd_ready(system_spiA_bmb_downSizer_io_input_cmd_ready),
    .io_input_cmd_payload_last(system_spiA_bmb_arbiter_io_output_cmd_payload_last),
    .io_input_cmd_payload_fragment_source(system_spiA_bmb_arbiter_io_output_cmd_payload_fragment_source),
    .io_input_cmd_payload_fragment_opcode(system_spiA_bmb_arbiter_io_output_cmd_payload_fragment_opcode),
    .io_input_cmd_payload_fragment_address(system_spiA_bmb_arbiter_io_output_cmd_payload_fragment_address),
    .io_input_cmd_payload_fragment_length(system_spiA_bmb_arbiter_io_output_cmd_payload_fragment_length),
    .io_input_cmd_payload_fragment_context(system_spiA_bmb_arbiter_io_output_cmd_payload_fragment_context),
    .io_input_rsp_valid(system_spiA_bmb_downSizer_io_input_rsp_valid),
    .io_input_rsp_ready(system_spiA_bmb_arbiter_io_output_rsp_ready),
    .io_input_rsp_payload_last(system_spiA_bmb_downSizer_io_input_rsp_payload_last),
    .io_input_rsp_payload_fragment_source(system_spiA_bmb_downSizer_io_input_rsp_payload_fragment_source),
    .io_input_rsp_payload_fragment_opcode(system_spiA_bmb_downSizer_io_input_rsp_payload_fragment_opcode),
    .io_input_rsp_payload_fragment_data(system_spiA_bmb_downSizer_io_input_rsp_payload_fragment_data),
    .io_input_rsp_payload_fragment_context(system_spiA_bmb_downSizer_io_input_rsp_payload_fragment_context),
    .io_output_cmd_valid(system_spiA_bmb_downSizer_io_output_cmd_valid),
    .io_output_cmd_ready(generator_cmd_ready),
    .io_output_cmd_payload_last(system_spiA_bmb_downSizer_io_output_cmd_payload_last),
    .io_output_cmd_payload_fragment_opcode(system_spiA_bmb_downSizer_io_output_cmd_payload_fragment_opcode),
    .io_output_cmd_payload_fragment_address(system_spiA_bmb_downSizer_io_output_cmd_payload_fragment_address),
    .io_output_cmd_payload_fragment_length(system_spiA_bmb_downSizer_io_output_cmd_payload_fragment_length),
    .io_output_cmd_payload_fragment_context(system_spiA_bmb_downSizer_io_output_cmd_payload_fragment_context),
    .io_output_rsp_valid(generator_rsp_valid),
    .io_output_rsp_ready(system_spiA_bmb_downSizer_io_output_rsp_ready),
    .io_output_rsp_payload_last(generator_rsp_payload_last),
    .io_output_rsp_payload_fragment_opcode(generator_rsp_payload_fragment_opcode),
    .io_output_rsp_payload_fragment_data(generator_rsp_payload_fragment_data),
    .io_output_rsp_payload_fragment_context(generator_rsp_payload_fragment_context),
    .clk_12M(clk_12M),
    .clockCtrl_systemReset(clockCtrl_systemReset) 
  );
  Apb3Decoder generator_decoder ( 
    .io_input_PADDR(generator_PADDR),
    .io_input_PSEL(generator_PSEL),
    .io_input_PENABLE(generator_PENABLE),
    .io_input_PREADY(generator_decoder_io_input_PREADY),
    .io_input_PWRITE(generator_PWRITE),
    .io_input_PWDATA(generator_PWDATA),
    .io_input_PRDATA(generator_decoder_io_input_PRDATA),
    .io_input_PSLVERROR(generator_decoder_io_input_PSLVERROR),
    .io_output_PADDR(generator_decoder_io_output_PADDR),
    .io_output_PSEL(generator_decoder_io_output_PSEL),
    .io_output_PENABLE(generator_decoder_io_output_PENABLE),
    .io_output_PREADY(apb3Router_1__io_input_PREADY),
    .io_output_PWRITE(generator_decoder_io_output_PWRITE),
    .io_output_PWDATA(generator_decoder_io_output_PWDATA),
    .io_output_PRDATA(apb3Router_1__io_input_PRDATA),
    .io_output_PSLVERROR(apb3Router_1__io_input_PSLVERROR) 
  );
  Apb3Router apb3Router_1_ ( 
    .io_input_PADDR(generator_decoder_io_output_PADDR),
    .io_input_PSEL(generator_decoder_io_output_PSEL),
    .io_input_PENABLE(generator_decoder_io_output_PENABLE),
    .io_input_PREADY(apb3Router_1__io_input_PREADY),
    .io_input_PWRITE(generator_decoder_io_output_PWRITE),
    .io_input_PWDATA(generator_decoder_io_output_PWDATA),
    .io_input_PRDATA(apb3Router_1__io_input_PRDATA),
    .io_input_PSLVERROR(apb3Router_1__io_input_PSLVERROR),
    .io_outputs_0_PADDR(apb3Router_1__io_outputs_0_PADDR),
    .io_outputs_0_PSEL(apb3Router_1__io_outputs_0_PSEL),
    .io_outputs_0_PENABLE(apb3Router_1__io_outputs_0_PENABLE),
    .io_outputs_0_PREADY(system_plic_apb_PREADY),
    .io_outputs_0_PWRITE(apb3Router_1__io_outputs_0_PWRITE),
    .io_outputs_0_PWDATA(apb3Router_1__io_outputs_0_PWDATA),
    .io_outputs_0_PRDATA(system_plic_apb_PRDATA),
    .io_outputs_0_PSLVERROR(system_plic_apb_PSLVERROR),
    .io_outputs_1_PADDR(apb3Router_1__io_outputs_1_PADDR),
    .io_outputs_1_PSEL(apb3Router_1__io_outputs_1_PSEL),
    .io_outputs_1_PENABLE(apb3Router_1__io_outputs_1_PENABLE),
    .io_outputs_1_PREADY(system_uartA_io_apb_PREADY),
    .io_outputs_1_PWRITE(apb3Router_1__io_outputs_1_PWRITE),
    .io_outputs_1_PWDATA(apb3Router_1__io_outputs_1_PWDATA),
    .io_outputs_1_PRDATA(system_uartA_io_apb_PRDATA),
    .io_outputs_1_PSLVERROR(_zz_70_),
    .io_outputs_2_PADDR(apb3Router_1__io_outputs_2_PADDR),
    .io_outputs_2_PSEL(apb3Router_1__io_outputs_2_PSEL),
    .io_outputs_2_PENABLE(apb3Router_1__io_outputs_2_PENABLE),
    .io_outputs_2_PREADY(system_gpioA_io_bus_PREADY),
    .io_outputs_2_PWRITE(apb3Router_1__io_outputs_2_PWRITE),
    .io_outputs_2_PWDATA(apb3Router_1__io_outputs_2_PWDATA),
    .io_outputs_2_PRDATA(system_gpioA_io_bus_PRDATA),
    .io_outputs_2_PSLVERROR(system_gpioA_io_bus_PSLVERROR),
    .io_outputs_3_PADDR(apb3Router_1__io_outputs_3_PADDR),
    .io_outputs_3_PSEL(apb3Router_1__io_outputs_3_PSEL),
    .io_outputs_3_PENABLE(apb3Router_1__io_outputs_3_PENABLE),
    .io_outputs_3_PREADY(system_spiA_io_apb_PREADY),
    .io_outputs_3_PWRITE(apb3Router_1__io_outputs_3_PWRITE),
    .io_outputs_3_PWDATA(apb3Router_1__io_outputs_3_PWDATA),
    .io_outputs_3_PRDATA(system_spiA_io_apb_PRDATA),
    .io_outputs_3_PSLVERROR(_zz_71_),
    .io_outputs_4_PADDR(apb3Router_1__io_outputs_4_PADDR),
    .io_outputs_4_PSEL(apb3Router_1__io_outputs_4_PSEL),
    .io_outputs_4_PENABLE(apb3Router_1__io_outputs_4_PENABLE),
    .io_outputs_4_PREADY(system_machineTimer_io_bus_PREADY),
    .io_outputs_4_PWRITE(apb3Router_1__io_outputs_4_PWRITE),
    .io_outputs_4_PWDATA(apb3Router_1__io_outputs_4_PWDATA),
    .io_outputs_4_PRDATA(system_machineTimer_io_bus_PRDATA),
    .io_outputs_4_PSLVERROR(system_machineTimer_io_bus_PSLVERROR),
    .clk_12M(clk_12M),
    .clockCtrl_systemReset(clockCtrl_systemReset) 
  );
  BmbToApb3Bridge system_peripheralBridge_bridge ( 
    .io_input_cmd_valid(system_peripheralBridge_input_burstUnburstifier_io_output_cmd_valid),
    .io_input_cmd_ready(system_peripheralBridge_bridge_io_input_cmd_ready),
    .io_input_cmd_payload_last(system_peripheralBridge_input_burstUnburstifier_io_output_cmd_payload_last),
    .io_input_cmd_payload_fragment_source(system_peripheralBridge_input_burstUnburstifier_io_output_cmd_payload_fragment_source),
    .io_input_cmd_payload_fragment_opcode(system_peripheralBridge_input_burstUnburstifier_io_output_cmd_payload_fragment_opcode),
    .io_input_cmd_payload_fragment_address(system_peripheralBridge_input_burstUnburstifier_io_output_cmd_payload_fragment_address),
    .io_input_cmd_payload_fragment_length(system_peripheralBridge_input_burstUnburstifier_io_output_cmd_payload_fragment_length),
    .io_input_cmd_payload_fragment_data(system_peripheralBridge_input_burstUnburstifier_io_output_cmd_payload_fragment_data),
    .io_input_cmd_payload_fragment_mask(system_peripheralBridge_input_burstUnburstifier_io_output_cmd_payload_fragment_mask),
    .io_input_cmd_payload_fragment_context(system_peripheralBridge_input_burstUnburstifier_io_output_cmd_payload_fragment_context),
    .io_input_rsp_valid(system_peripheralBridge_bridge_io_input_rsp_valid),
    .io_input_rsp_ready(system_peripheralBridge_input_burstUnburstifier_io_output_rsp_ready),
    .io_input_rsp_payload_last(system_peripheralBridge_bridge_io_input_rsp_payload_last),
    .io_input_rsp_payload_fragment_source(system_peripheralBridge_bridge_io_input_rsp_payload_fragment_source),
    .io_input_rsp_payload_fragment_opcode(system_peripheralBridge_bridge_io_input_rsp_payload_fragment_opcode),
    .io_input_rsp_payload_fragment_data(system_peripheralBridge_bridge_io_input_rsp_payload_fragment_data),
    .io_input_rsp_payload_fragment_context(system_peripheralBridge_bridge_io_input_rsp_payload_fragment_context),
    .io_output_PADDR(system_peripheralBridge_bridge_io_output_PADDR),
    .io_output_PSEL(system_peripheralBridge_bridge_io_output_PSEL),
    .io_output_PENABLE(system_peripheralBridge_bridge_io_output_PENABLE),
    .io_output_PREADY(generator_PREADY),
    .io_output_PWRITE(system_peripheralBridge_bridge_io_output_PWRITE),
    .io_output_PWDATA(system_peripheralBridge_bridge_io_output_PWDATA),
    .io_output_PRDATA(generator_PRDATA),
    .io_output_PSLVERROR(generator_PSLVERROR),
    .clk_12M(clk_12M),
    .clockCtrl_systemReset(clockCtrl_systemReset) 
  );
  BmbArbiter_3_ system_peripheralBridge_input_arbiter ( 
    .io_inputs_0_cmd_valid(system_bridge_bmb_decoder_io_outputs_1_cmd_valid),
    .io_inputs_0_cmd_ready(system_peripheralBridge_input_arbiter_io_inputs_0_cmd_ready),
    .io_inputs_0_cmd_payload_last(system_bridge_bmb_decoder_io_outputs_1_cmd_payload_last),
    .io_inputs_0_cmd_payload_fragment_source(system_bridge_bmb_decoder_io_outputs_1_cmd_payload_fragment_source),
    .io_inputs_0_cmd_payload_fragment_opcode(system_bridge_bmb_decoder_io_outputs_1_cmd_payload_fragment_opcode),
    .io_inputs_0_cmd_payload_fragment_address(_zz_72_),
    .io_inputs_0_cmd_payload_fragment_length(system_bridge_bmb_decoder_io_outputs_1_cmd_payload_fragment_length),
    .io_inputs_0_cmd_payload_fragment_data(system_bridge_bmb_decoder_io_outputs_1_cmd_payload_fragment_data),
    .io_inputs_0_cmd_payload_fragment_mask(system_bridge_bmb_decoder_io_outputs_1_cmd_payload_fragment_mask),
    .io_inputs_0_cmd_payload_fragment_context(system_bridge_bmb_decoder_io_outputs_1_cmd_payload_fragment_context),
    .io_inputs_0_rsp_valid(system_peripheralBridge_input_arbiter_io_inputs_0_rsp_valid),
    .io_inputs_0_rsp_ready(system_bridge_bmb_decoder_io_outputs_1_rsp_ready),
    .io_inputs_0_rsp_payload_last(system_peripheralBridge_input_arbiter_io_inputs_0_rsp_payload_last),
    .io_inputs_0_rsp_payload_fragment_source(system_peripheralBridge_input_arbiter_io_inputs_0_rsp_payload_fragment_source),
    .io_inputs_0_rsp_payload_fragment_opcode(system_peripheralBridge_input_arbiter_io_inputs_0_rsp_payload_fragment_opcode),
    .io_inputs_0_rsp_payload_fragment_data(system_peripheralBridge_input_arbiter_io_inputs_0_rsp_payload_fragment_data),
    .io_inputs_0_rsp_payload_fragment_context(system_peripheralBridge_input_arbiter_io_inputs_0_rsp_payload_fragment_context),
    .io_output_cmd_valid(system_peripheralBridge_input_arbiter_io_output_cmd_valid),
    .io_output_cmd_ready(system_peripheralBridge_input_burstUnburstifier_io_input_cmd_ready),
    .io_output_cmd_payload_last(system_peripheralBridge_input_arbiter_io_output_cmd_payload_last),
    .io_output_cmd_payload_fragment_source(system_peripheralBridge_input_arbiter_io_output_cmd_payload_fragment_source),
    .io_output_cmd_payload_fragment_opcode(system_peripheralBridge_input_arbiter_io_output_cmd_payload_fragment_opcode),
    .io_output_cmd_payload_fragment_address(system_peripheralBridge_input_arbiter_io_output_cmd_payload_fragment_address),
    .io_output_cmd_payload_fragment_length(system_peripheralBridge_input_arbiter_io_output_cmd_payload_fragment_length),
    .io_output_cmd_payload_fragment_data(system_peripheralBridge_input_arbiter_io_output_cmd_payload_fragment_data),
    .io_output_cmd_payload_fragment_mask(system_peripheralBridge_input_arbiter_io_output_cmd_payload_fragment_mask),
    .io_output_cmd_payload_fragment_context(system_peripheralBridge_input_arbiter_io_output_cmd_payload_fragment_context),
    .io_output_rsp_valid(system_peripheralBridge_input_burstUnburstifier_io_input_rsp_valid),
    .io_output_rsp_ready(system_peripheralBridge_input_arbiter_io_output_rsp_ready),
    .io_output_rsp_payload_last(system_peripheralBridge_input_burstUnburstifier_io_input_rsp_payload_last),
    .io_output_rsp_payload_fragment_source(system_peripheralBridge_input_burstUnburstifier_io_input_rsp_payload_fragment_source),
    .io_output_rsp_payload_fragment_opcode(system_peripheralBridge_input_burstUnburstifier_io_input_rsp_payload_fragment_opcode),
    .io_output_rsp_payload_fragment_data(system_peripheralBridge_input_burstUnburstifier_io_input_rsp_payload_fragment_data),
    .io_output_rsp_payload_fragment_context(system_peripheralBridge_input_burstUnburstifier_io_input_rsp_payload_fragment_context) 
  );
  BmbUnburstify_1_ system_peripheralBridge_input_burstUnburstifier ( 
    .io_input_cmd_valid(system_peripheralBridge_input_arbiter_io_output_cmd_valid),
    .io_input_cmd_ready(system_peripheralBridge_input_burstUnburstifier_io_input_cmd_ready),
    .io_input_cmd_payload_last(system_peripheralBridge_input_arbiter_io_output_cmd_payload_last),
    .io_input_cmd_payload_fragment_source(system_peripheralBridge_input_arbiter_io_output_cmd_payload_fragment_source),
    .io_input_cmd_payload_fragment_opcode(system_peripheralBridge_input_arbiter_io_output_cmd_payload_fragment_opcode),
    .io_input_cmd_payload_fragment_address(system_peripheralBridge_input_arbiter_io_output_cmd_payload_fragment_address),
    .io_input_cmd_payload_fragment_length(system_peripheralBridge_input_arbiter_io_output_cmd_payload_fragment_length),
    .io_input_cmd_payload_fragment_data(system_peripheralBridge_input_arbiter_io_output_cmd_payload_fragment_data),
    .io_input_cmd_payload_fragment_mask(system_peripheralBridge_input_arbiter_io_output_cmd_payload_fragment_mask),
    .io_input_cmd_payload_fragment_context(system_peripheralBridge_input_arbiter_io_output_cmd_payload_fragment_context),
    .io_input_rsp_valid(system_peripheralBridge_input_burstUnburstifier_io_input_rsp_valid),
    .io_input_rsp_ready(system_peripheralBridge_input_arbiter_io_output_rsp_ready),
    .io_input_rsp_payload_last(system_peripheralBridge_input_burstUnburstifier_io_input_rsp_payload_last),
    .io_input_rsp_payload_fragment_source(system_peripheralBridge_input_burstUnburstifier_io_input_rsp_payload_fragment_source),
    .io_input_rsp_payload_fragment_opcode(system_peripheralBridge_input_burstUnburstifier_io_input_rsp_payload_fragment_opcode),
    .io_input_rsp_payload_fragment_data(system_peripheralBridge_input_burstUnburstifier_io_input_rsp_payload_fragment_data),
    .io_input_rsp_payload_fragment_context(system_peripheralBridge_input_burstUnburstifier_io_input_rsp_payload_fragment_context),
    .io_output_cmd_valid(system_peripheralBridge_input_burstUnburstifier_io_output_cmd_valid),
    .io_output_cmd_ready(system_peripheralBridge_bridge_io_input_cmd_ready),
    .io_output_cmd_payload_last(system_peripheralBridge_input_burstUnburstifier_io_output_cmd_payload_last),
    .io_output_cmd_payload_fragment_source(system_peripheralBridge_input_burstUnburstifier_io_output_cmd_payload_fragment_source),
    .io_output_cmd_payload_fragment_opcode(system_peripheralBridge_input_burstUnburstifier_io_output_cmd_payload_fragment_opcode),
    .io_output_cmd_payload_fragment_address(system_peripheralBridge_input_burstUnburstifier_io_output_cmd_payload_fragment_address),
    .io_output_cmd_payload_fragment_length(system_peripheralBridge_input_burstUnburstifier_io_output_cmd_payload_fragment_length),
    .io_output_cmd_payload_fragment_data(system_peripheralBridge_input_burstUnburstifier_io_output_cmd_payload_fragment_data),
    .io_output_cmd_payload_fragment_mask(system_peripheralBridge_input_burstUnburstifier_io_output_cmd_payload_fragment_mask),
    .io_output_cmd_payload_fragment_context(system_peripheralBridge_input_burstUnburstifier_io_output_cmd_payload_fragment_context),
    .io_output_rsp_valid(system_peripheralBridge_bridge_io_input_rsp_valid),
    .io_output_rsp_ready(system_peripheralBridge_input_burstUnburstifier_io_output_rsp_ready),
    .io_output_rsp_payload_last(system_peripheralBridge_bridge_io_input_rsp_payload_last),
    .io_output_rsp_payload_fragment_source(system_peripheralBridge_bridge_io_input_rsp_payload_fragment_source),
    .io_output_rsp_payload_fragment_opcode(system_peripheralBridge_bridge_io_input_rsp_payload_fragment_opcode),
    .io_output_rsp_payload_fragment_data(system_peripheralBridge_bridge_io_input_rsp_payload_fragment_data),
    .io_output_rsp_payload_fragment_context(system_peripheralBridge_bridge_io_input_rsp_payload_fragment_context),
    .clk_12M(clk_12M),
    .clockCtrl_systemReset(clockCtrl_systemReset) 
  );
  SB_IO #( 
    .PIN_TYPE((6'b101001)) 
  ) sB_IO_5_ ( 
    .PACKAGE_PIN(system_gpioA_gpio_0),
    .OUTPUT_ENABLE(_zz_73_),
    .D_OUT_0(_zz_74_),
    .D_IN_0(sB_IO_5__D_IN_0) 
  );
  SB_IO #( 
    .PIN_TYPE((6'b101001)) 
  ) sB_IO_6_ ( 
    .PACKAGE_PIN(system_gpioA_gpio_1),
    .OUTPUT_ENABLE(_zz_75_),
    .D_OUT_0(_zz_76_),
    .D_IN_0(sB_IO_6__D_IN_0) 
  );
  SB_IO #( 
    .PIN_TYPE((6'b101001)) 
  ) sB_IO_7_ ( 
    .PACKAGE_PIN(system_gpioA_gpio_2),
    .OUTPUT_ENABLE(_zz_77_),
    .D_OUT_0(_zz_78_),
    .D_IN_0(sB_IO_7__D_IN_0) 
  );
  SB_IO #( 
    .PIN_TYPE((6'b101001)) 
  ) sB_IO_8_ ( 
    .PACKAGE_PIN(system_gpioA_gpio_3),
    .OUTPUT_ENABLE(_zz_79_),
    .D_OUT_0(_zz_80_),
    .D_IN_0(sB_IO_8__D_IN_0) 
  );
  assign clockCtrl_inputResetTrigger = 1'b0;
  always @ (*) begin
    clockCtrl_resetUnbuffered = 1'b0;
    if(_zz_81_)begin
      clockCtrl_resetUnbuffered = 1'b1;
    end
  end

  always @ (*) begin
    clockCtrl_resetRequest = 1'b0;
    if(system_cpu_cpu_debug_resetOut_regNext)begin
      clockCtrl_resetRequest = 1'b1;
    end
  end

  assign system_uartA_uart_txd = system_uartA_io_uart_txd;
  assign _zz_2_ = system_gpioA_io_gpio_write;
  assign _zz_3_ = system_gpioA_io_gpio_writeEnable;
  assign _zz_40_ = (system_cpu_iBus_decoder_io_input_rsp_payload_fragment_opcode == (1'b1));
  assign _zz_4_[0] = system_cpu_cpu_dBus_cmd_payload_wr;
  always @ (*) begin
    case(system_cpu_cpu_dBus_cmd_payload_size)
      2'b00 : begin
        _zz_5_ = (2'b00);
      end
      2'b01 : begin
        _zz_5_ = (2'b01);
      end
      default : begin
        _zz_5_ = (2'b11);
      end
    endcase
  end

  always @ (*) begin
    case(system_cpu_cpu_dBus_cmd_payload_size)
      2'b00 : begin
        _zz_6_ = (4'b0001);
      end
      2'b01 : begin
        _zz_6_ = (4'b0011);
      end
      default : begin
        _zz_6_ = (4'b1111);
      end
    endcase
  end

  assign _zz_41_ = (system_cpu_dBus_decoder_io_input_rsp_valid && (! system_cpu_dBus_decoder_io_input_rsp_payload_fragment_context[0]));
  assign _zz_42_ = (system_cpu_dBus_decoder_io_input_rsp_payload_fragment_opcode == (1'b1));
  assign _zz_39_ = systemDebugger_1__io_mem_cmd_payload_address[7:0];
  assign system_cpu_tdo = jtagBridge_1__io_jtag_tdo;
  assign _zz_46_ = 1'b1;
  assign _zz_43_ = 1'b1;
  assign _zz_44_ = (1'b0);
  assign _zz_45_ = (5'b11111);
  assign _zz_50_ = 1'b1;
  assign _zz_47_ = 1'b1;
  assign _zz_48_ = (system_cpu_cpu_dBus_cmd_payload_wr ? (1'b1) : (1'b0));
  assign _zz_49_ = (_zz_6_ <<< system_cpu_cpu_dBus_cmd_payload_address[1 : 0]);
  assign system_spiA_phy_sclk_write = system_spiA_io_spi_sclk_write;
  assign system_spiA_phy_data_0_writeEnable = system_spiA_io_spi_data_0_writeEnable;
  assign system_spiA_phy_data_0_write = system_spiA_io_spi_data_0_write;
  assign system_spiA_phy_data_1_writeEnable = system_spiA_io_spi_data_1_writeEnable;
  assign system_spiA_phy_data_1_write = system_spiA_io_spi_data_1_write;
  assign system_spiA_phy_ss = system_spiA_io_spi_ss;
  assign _zz_52_ = 1'b1;
  assign _zz_55_ = system_spiA_phy_ss[0];
  assign _zz_56_ = 1'b1;
  assign _zz_59_ = system_spiA_phy_sclk_write[0];
  assign _zz_60_ = 1'b1;
  assign _zz_61_ = system_spiA_phy_data_0_write[0];
  always @ (*) begin
    system_spiA_phy_data_0_read[0] = sB_IO_3__D_IN_0;
    system_spiA_phy_data_0_read[1] = sB_IO_3__D_IN_1_regNext;
  end

  assign _zz_62_ = 1'b1;
  assign _zz_63_ = system_spiA_phy_data_1_write[0];
  always @ (*) begin
    system_spiA_phy_data_1_read[0] = sB_IO_4__D_IN_0;
    system_spiA_phy_data_1_read[1] = sB_IO_4__D_IN_1_regNext;
  end

  assign system_bridge_cmd_valid = system_bridge_bmb_arbiter_io_output_cmd_valid;
  assign system_bridge_rsp_ready = system_bridge_bmb_arbiter_io_output_rsp_ready;
  assign system_bridge_cmd_payload_last = system_bridge_bmb_arbiter_io_output_cmd_payload_last;
  assign system_bridge_cmd_payload_fragment_source = system_bridge_bmb_arbiter_io_output_cmd_payload_fragment_source;
  assign system_bridge_cmd_payload_fragment_opcode = system_bridge_bmb_arbiter_io_output_cmd_payload_fragment_opcode;
  assign system_bridge_cmd_payload_fragment_address = system_bridge_bmb_arbiter_io_output_cmd_payload_fragment_address;
  assign system_bridge_cmd_payload_fragment_length = system_bridge_bmb_arbiter_io_output_cmd_payload_fragment_length;
  assign system_bridge_cmd_payload_fragment_data = system_bridge_bmb_arbiter_io_output_cmd_payload_fragment_data;
  assign system_bridge_cmd_payload_fragment_mask = system_bridge_bmb_arbiter_io_output_cmd_payload_fragment_mask;
  assign system_bridge_cmd_payload_fragment_context = system_bridge_bmb_arbiter_io_output_cmd_payload_fragment_context;
  assign _zz_65_ = (32'bxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx);
  assign _zz_66_ = (4'bxxxx);
  assign _zz_67_ = (1'b0);
  assign _zz_64_ = {3'd0, system_cpu_dBus_decoder_io_outputs_0_cmd_payload_fragment_length};
  assign system_plic_targets_0_gateways_1_source = system_gpioA_io_interrupt[0];
  assign system_plic_targets_0_gateways_2_source = system_gpioA_io_interrupt[1];
  assign generator = system_gpioA_io_interrupt[2];
  assign generator_1_ = system_gpioA_io_interrupt[3];
  assign generator_cmd_ready = system_spiA_io_xip_cmd_ready;
  assign generator_rsp_valid = system_spiA_io_xip_rsp_valid;
  assign generator_rsp_payload_fragment_data = system_spiA_io_xip_rsp_payload_fragment;
  assign generator_rsp_payload_last = system_spiA_io_xip_rsp_payload_last;
  assign generator_rsp_payload_fragment_context = generator_cmd_payload_fragment_context_regNextWhen;
  assign generator_rsp_payload_fragment_opcode = (1'b0);
  assign system_plic_apb_PREADY = 1'b1;
  always @ (*) begin
    system_plic_apb_PRDATA = (32'b00000000000000000000000000000000);
    case(system_plic_apb_PADDR)
      22'b0000000000000000000100 : begin
        system_plic_apb_PRDATA[1 : 0] = system_plic_targets_0_gateways_0_priority;
      end
      22'b0000000001000000000100 : begin
        system_plic_apb_PRDATA[0 : 0] = system_plic_targets_0_gateways_0_ip;
      end
      22'b0000000000000000010000 : begin
        system_plic_apb_PRDATA[1 : 0] = system_plic_targets_0_gateways_1_priority;
      end
      22'b0000000001000000010000 : begin
        system_plic_apb_PRDATA[0 : 0] = system_plic_targets_0_gateways_1_ip;
      end
      22'b0000000000000000010100 : begin
        system_plic_apb_PRDATA[1 : 0] = system_plic_targets_0_gateways_2_priority;
      end
      22'b0000000001000000010100 : begin
        system_plic_apb_PRDATA[0 : 0] = system_plic_targets_0_gateways_2_ip;
      end
      22'b1000000000000000000000 : begin
        system_plic_apb_PRDATA[1 : 0] = system_plic_targets_0_threshold;
      end
      22'b1000000000000000000100 : begin
        system_plic_apb_PRDATA[2 : 0] = system_plic_targets_0_claim;
      end
      22'b0000000010000000000000 : begin
        system_plic_apb_PRDATA[1 : 1] = system_plic_targets_0_ie_0;
        system_plic_apb_PRDATA[4 : 4] = system_plic_targets_0_ie_1;
        system_plic_apb_PRDATA[5 : 5] = system_plic_targets_0_ie_2;
      end
      default : begin
      end
    endcase
  end

  assign system_plic_apb_PSLVERROR = 1'b0;
  assign system_plic_bus_askWrite = ((system_plic_apb_PSEL[0] && system_plic_apb_PENABLE) && system_plic_apb_PWRITE);
  assign system_plic_bus_askRead = ((system_plic_apb_PSEL[0] && system_plic_apb_PENABLE) && (! system_plic_apb_PWRITE));
  assign system_plic_bus_doWrite = (((system_plic_apb_PSEL[0] && system_plic_apb_PENABLE) && system_plic_apb_PREADY) && system_plic_apb_PWRITE);
  assign system_plic_bus_doRead = (((system_plic_apb_PSEL[0] && system_plic_apb_PENABLE) && system_plic_apb_PREADY) && (! system_plic_apb_PWRITE));
  assign system_plic_targets_0_requests_0_priority = (2'b00);
  assign system_plic_targets_0_requests_0_id = (3'b000);
  assign system_plic_targets_0_requests_0_valid = 1'b1;
  assign system_plic_targets_0_requests_1_priority = system_plic_targets_0_gateways_0_priority;
  assign system_plic_targets_0_requests_1_id = (3'b001);
  assign system_plic_targets_0_requests_1_valid = (system_plic_targets_0_gateways_0_ip && system_plic_targets_0_ie_0);
  assign system_plic_targets_0_requests_2_priority = system_plic_targets_0_gateways_1_priority;
  assign system_plic_targets_0_requests_2_id = (3'b100);
  assign system_plic_targets_0_requests_2_valid = (system_plic_targets_0_gateways_1_ip && system_plic_targets_0_ie_1);
  assign system_plic_targets_0_requests_3_priority = system_plic_targets_0_gateways_2_priority;
  assign system_plic_targets_0_requests_3_id = (3'b101);
  assign system_plic_targets_0_requests_3_valid = (system_plic_targets_0_gateways_2_ip && system_plic_targets_0_ie_2);
  assign _zz_12_ = ((! system_plic_targets_0_requests_1_valid) || (system_plic_targets_0_requests_0_valid && (system_plic_targets_0_requests_1_priority <= system_plic_targets_0_requests_0_priority)));
  assign _zz_13_ = (_zz_12_ ? system_plic_targets_0_requests_0_priority : system_plic_targets_0_requests_1_priority);
  assign _zz_14_ = (_zz_12_ ? system_plic_targets_0_requests_0_valid : system_plic_targets_0_requests_1_valid);
  assign _zz_15_ = ((! system_plic_targets_0_requests_3_valid) || (system_plic_targets_0_requests_2_valid && (system_plic_targets_0_requests_3_priority <= system_plic_targets_0_requests_2_priority)));
  assign _zz_16_ = (_zz_15_ ? system_plic_targets_0_requests_2_priority : system_plic_targets_0_requests_3_priority);
  assign _zz_17_ = (_zz_15_ ? system_plic_targets_0_requests_2_valid : system_plic_targets_0_requests_3_valid);
  assign _zz_18_ = ((! _zz_17_) || (_zz_14_ && (_zz_16_ <= _zz_13_)));
  assign system_plic_targets_0_bestRequest_priority = (_zz_18_ ? _zz_13_ : _zz_16_);
  assign system_plic_targets_0_bestRequest_id = (_zz_18_ ? (_zz_12_ ? system_plic_targets_0_requests_0_id : system_plic_targets_0_requests_1_id) : (_zz_15_ ? system_plic_targets_0_requests_2_id : system_plic_targets_0_requests_3_id));
  assign system_plic_targets_0_bestRequest_valid = (_zz_18_ ? _zz_14_ : _zz_17_);
  assign system_plic_targets_0_iep = (system_plic_targets_0_threshold < system_plic_targets_0_bestRequest_priority);
  assign system_plic_targets_0_claim = (system_plic_targets_0_iep ? system_plic_targets_0_bestRequest_id : (3'b000));
  assign system_plic_targets_0_gateways_0_priority = _zz_19_;
  assign system_plic_targets_0_gateways_1_priority = _zz_20_;
  assign system_plic_targets_0_gateways_2_priority = _zz_21_;
  always @ (*) begin
    system_plic_bridge_claim_valid = 1'b0;
    case(system_plic_apb_PADDR)
      22'b0000000000000000000100 : begin
      end
      22'b0000000001000000000100 : begin
      end
      22'b0000000000000000010000 : begin
      end
      22'b0000000001000000010000 : begin
      end
      22'b0000000000000000010100 : begin
      end
      22'b0000000001000000010100 : begin
      end
      22'b1000000000000000000000 : begin
      end
      22'b1000000000000000000100 : begin
        if(system_plic_bus_doRead)begin
          system_plic_bridge_claim_valid = 1'b1;
        end
      end
      22'b0000000010000000000000 : begin
      end
      default : begin
      end
    endcase
  end

  always @ (*) begin
    system_plic_bridge_claim_payload = (3'bxxx);
    case(system_plic_apb_PADDR)
      22'b0000000000000000000100 : begin
      end
      22'b0000000001000000000100 : begin
      end
      22'b0000000000000000010000 : begin
      end
      22'b0000000001000000010000 : begin
      end
      22'b0000000000000000010100 : begin
      end
      22'b0000000001000000010100 : begin
      end
      22'b1000000000000000000000 : begin
      end
      22'b1000000000000000000100 : begin
        if(system_plic_bus_doRead)begin
          system_plic_bridge_claim_payload = system_plic_targets_0_claim;
        end
      end
      22'b0000000010000000000000 : begin
      end
      default : begin
      end
    endcase
  end

  always @ (*) begin
    system_plic_bridge_completion_valid = 1'b0;
    if(system_plic_bridge_targetMapping_0_targetCompletion_valid)begin
      system_plic_bridge_completion_valid = 1'b1;
    end
  end

  always @ (*) begin
    system_plic_bridge_completion_payload = (3'bxxx);
    if(system_plic_bridge_targetMapping_0_targetCompletion_valid)begin
      system_plic_bridge_completion_payload = system_plic_bridge_targetMapping_0_targetCompletion_payload;
    end
  end

  assign system_plic_targets_0_threshold = _zz_22_;
  always @ (*) begin
    system_plic_bridge_targetMapping_0_targetCompletion_valid = 1'b0;
    case(system_plic_apb_PADDR)
      22'b0000000000000000000100 : begin
      end
      22'b0000000001000000000100 : begin
      end
      22'b0000000000000000010000 : begin
      end
      22'b0000000001000000010000 : begin
      end
      22'b0000000000000000010100 : begin
      end
      22'b0000000001000000010100 : begin
      end
      22'b1000000000000000000000 : begin
      end
      22'b1000000000000000000100 : begin
        if(system_plic_bus_doWrite)begin
          system_plic_bridge_targetMapping_0_targetCompletion_valid = 1'b1;
        end
      end
      22'b0000000010000000000000 : begin
      end
      default : begin
      end
    endcase
  end

  assign system_plic_targets_0_ie_0 = _zz_23_;
  assign system_plic_targets_0_ie_1 = _zz_24_;
  assign system_plic_targets_0_ie_2 = _zz_25_;
  assign system_bridge_cmd_ready = ((1'b1 && (! system_bridge_cmd_m2sPipe_valid)) || system_bridge_cmd_m2sPipe_ready);
  assign system_bridge_cmd_m2sPipe_valid = _zz_26_;
  assign system_bridge_cmd_m2sPipe_payload_last = _zz_27_;
  assign system_bridge_cmd_m2sPipe_payload_fragment_source = _zz_28_;
  assign system_bridge_cmd_m2sPipe_payload_fragment_opcode = _zz_29_;
  assign system_bridge_cmd_m2sPipe_payload_fragment_address = _zz_30_;
  assign system_bridge_cmd_m2sPipe_payload_fragment_length = _zz_31_;
  assign system_bridge_cmd_m2sPipe_payload_fragment_data = _zz_32_;
  assign system_bridge_cmd_m2sPipe_payload_fragment_mask = _zz_33_;
  assign system_bridge_cmd_m2sPipe_payload_fragment_context = _zz_34_;
  assign system_bridge_cmd_m2sPipe_ready = system_bridge_bmb_decoder_io_input_cmd_ready;
  assign system_bridge_rsp_valid = system_bridge_bmb_decoder_io_input_rsp_valid;
  assign system_bridge_rsp_payload_last = system_bridge_bmb_decoder_io_input_rsp_payload_last;
  assign system_bridge_rsp_payload_fragment_source = system_bridge_bmb_decoder_io_input_rsp_payload_fragment_source;
  assign system_bridge_rsp_payload_fragment_opcode = system_bridge_bmb_decoder_io_input_rsp_payload_fragment_opcode;
  assign system_bridge_rsp_payload_fragment_data = system_bridge_bmb_decoder_io_input_rsp_payload_fragment_data;
  assign system_bridge_rsp_payload_fragment_context = system_bridge_bmb_decoder_io_input_rsp_payload_fragment_context;
  assign _zz_68_ = system_bridge_bmb_decoder_io_outputs_0_cmd_payload_fragment_address[16:0];
  assign generator_cmd_valid = system_spiA_bmb_downSizer_io_output_cmd_valid;
  assign generator_rsp_ready = system_spiA_bmb_downSizer_io_output_rsp_ready;
  assign generator_cmd_payload_last = system_spiA_bmb_downSizer_io_output_cmd_payload_last;
  assign generator_cmd_payload_fragment_opcode = system_spiA_bmb_downSizer_io_output_cmd_payload_fragment_opcode;
  assign generator_cmd_payload_fragment_address = system_spiA_bmb_downSizer_io_output_cmd_payload_fragment_address;
  assign generator_cmd_payload_fragment_length = system_spiA_bmb_downSizer_io_output_cmd_payload_fragment_length;
  assign generator_cmd_payload_fragment_context = system_spiA_bmb_downSizer_io_output_cmd_payload_fragment_context;
  assign generator_PREADY = generator_decoder_io_input_PREADY;
  assign generator_PRDATA = generator_decoder_io_input_PRDATA;
  assign generator_PSLVERROR = generator_decoder_io_input_PSLVERROR;
  assign system_plic_apb_PADDR = apb3Router_1__io_outputs_0_PADDR[21:0];
  assign system_plic_apb_PSEL = apb3Router_1__io_outputs_0_PSEL;
  assign system_plic_apb_PENABLE = apb3Router_1__io_outputs_0_PENABLE;
  assign system_plic_apb_PWRITE = apb3Router_1__io_outputs_0_PWRITE;
  assign system_plic_apb_PWDATA = apb3Router_1__io_outputs_0_PWDATA;
  assign _zz_35_ = apb3Router_1__io_outputs_1_PADDR[3:0];
  assign _zz_70_ = 1'b0;
  assign _zz_36_ = apb3Router_1__io_outputs_2_PADDR[11:0];
  assign _zz_51_ = apb3Router_1__io_outputs_3_PADDR[7:0];
  assign _zz_71_ = 1'b0;
  assign _zz_37_ = apb3Router_1__io_outputs_4_PADDR[3:0];
  assign generator_PADDR = system_peripheralBridge_bridge_io_output_PADDR;
  assign generator_PSEL = system_peripheralBridge_bridge_io_output_PSEL;
  assign generator_PENABLE = system_peripheralBridge_bridge_io_output_PENABLE;
  assign generator_PWRITE = system_peripheralBridge_bridge_io_output_PWRITE;
  assign generator_PWDATA = system_peripheralBridge_bridge_io_output_PWDATA;
  assign _zz_72_ = system_bridge_bmb_decoder_io_outputs_1_cmd_payload_fragment_address[23:0];
  assign _zz_69_ = system_bridge_bmb_decoder_io_outputs_2_cmd_payload_fragment_address[23:0];
  assign system_plic_bridge_targetMapping_0_targetCompletion_payload = system_plic_apb_PWDATA[2 : 0];
  assign _zz_73_ = _zz_3_[0];
  assign _zz_74_ = _zz_2_[0];
  always @ (*) begin
    _zz_1_[0] = sB_IO_5__D_IN_0;
    _zz_1_[1] = sB_IO_6__D_IN_0;
    _zz_1_[2] = sB_IO_7__D_IN_0;
    _zz_1_[3] = sB_IO_8__D_IN_0;
  end

  assign _zz_75_ = _zz_3_[1];
  assign _zz_76_ = _zz_2_[1];
  assign _zz_77_ = _zz_3_[2];
  assign _zz_78_ = _zz_2_[2];
  assign _zz_79_ = _zz_3_[3];
  assign _zz_80_ = _zz_2_[3];
  assign _zz_38_ = 1'b0;
  assign _zz_53_ = 1'b0;
  assign _zz_54_ = 1'b0;
  assign _zz_57_ = 1'b0;
  assign _zz_58_ = 1'b0;
  always @ (posedge clk_12M) begin
    if(_zz_81_)begin
      clockCtrl_holdingLogic_resetCounter <= (clockCtrl_holdingLogic_resetCounter + (8'b00000001));
    end
    if(clockCtrl_inputResetTrigger)begin
      clockCtrl_holdingLogic_resetCounter <= (8'b00000000);
    end
  end

  always @ (posedge clk_12M) begin
    clockCtrl_systemReset <= (clockCtrl_resetUnbuffered || bufferCC_4__io_dataOut);
    clockCtrl_resetUnbuffered_regNext <= clockCtrl_resetUnbuffered;
  end

  always @ (posedge clk_12M) begin
    if(clockCtrl_systemReset) begin
      system_plic_targets_0_gateways_0_ip <= 1'b0;
      system_plic_targets_0_gateways_0_waitCompletion <= 1'b0;
      system_plic_targets_0_gateways_1_ip <= 1'b0;
      system_plic_targets_0_gateways_1_waitCompletion <= 1'b0;
      system_plic_targets_0_gateways_2_ip <= 1'b0;
      system_plic_targets_0_gateways_2_waitCompletion <= 1'b0;
      _zz_19_ <= (2'b00);
      _zz_20_ <= (2'b00);
      _zz_21_ <= (2'b00);
      _zz_22_ <= (2'b00);
      _zz_23_ <= 1'b0;
      _zz_24_ <= 1'b0;
      _zz_25_ <= 1'b0;
      _zz_26_ <= 1'b0;
    end else begin
      if((! system_plic_targets_0_gateways_0_waitCompletion))begin
        system_plic_targets_0_gateways_0_ip <= system_uartA_io_interrupt;
        system_plic_targets_0_gateways_0_waitCompletion <= system_uartA_io_interrupt;
      end
      if((! system_plic_targets_0_gateways_1_waitCompletion))begin
        system_plic_targets_0_gateways_1_ip <= system_plic_targets_0_gateways_1_source;
        system_plic_targets_0_gateways_1_waitCompletion <= system_plic_targets_0_gateways_1_source;
      end
      if((! system_plic_targets_0_gateways_2_waitCompletion))begin
        system_plic_targets_0_gateways_2_ip <= system_plic_targets_0_gateways_2_source;
        system_plic_targets_0_gateways_2_waitCompletion <= system_plic_targets_0_gateways_2_source;
      end
      if(system_plic_bridge_claim_valid)begin
        case(system_plic_bridge_claim_payload)
          3'b001 : begin
            system_plic_targets_0_gateways_0_ip <= 1'b0;
          end
          3'b100 : begin
            system_plic_targets_0_gateways_1_ip <= 1'b0;
          end
          3'b101 : begin
            system_plic_targets_0_gateways_2_ip <= 1'b0;
          end
          default : begin
          end
        endcase
      end
      if(system_plic_bridge_completion_valid)begin
        case(system_plic_bridge_completion_payload)
          3'b001 : begin
            system_plic_targets_0_gateways_0_waitCompletion <= 1'b0;
          end
          3'b100 : begin
            system_plic_targets_0_gateways_1_waitCompletion <= 1'b0;
          end
          3'b101 : begin
            system_plic_targets_0_gateways_2_waitCompletion <= 1'b0;
          end
          default : begin
          end
        endcase
      end
      if(system_bridge_cmd_ready)begin
        _zz_26_ <= system_bridge_cmd_valid;
      end
      case(system_plic_apb_PADDR)
        22'b0000000000000000000100 : begin
          if(system_plic_bus_doWrite)begin
            _zz_19_ <= system_plic_apb_PWDATA[1 : 0];
          end
        end
        22'b0000000001000000000100 : begin
        end
        22'b0000000000000000010000 : begin
          if(system_plic_bus_doWrite)begin
            _zz_20_ <= system_plic_apb_PWDATA[1 : 0];
          end
        end
        22'b0000000001000000010000 : begin
        end
        22'b0000000000000000010100 : begin
          if(system_plic_bus_doWrite)begin
            _zz_21_ <= system_plic_apb_PWDATA[1 : 0];
          end
        end
        22'b0000000001000000010100 : begin
        end
        22'b1000000000000000000000 : begin
          if(system_plic_bus_doWrite)begin
            _zz_22_ <= system_plic_apb_PWDATA[1 : 0];
          end
        end
        22'b1000000000000000000100 : begin
        end
        22'b0000000010000000000000 : begin
          if(system_plic_bus_doWrite)begin
            _zz_23_ <= _zz_82_[0];
            _zz_24_ <= _zz_83_[0];
            _zz_25_ <= _zz_84_[0];
          end
        end
        default : begin
        end
      endcase
    end
  end

  always @ (posedge clk_12M) begin
    system_cpu_cpu_debug_resetOut_regNext <= system_cpu_cpu_debug_resetOut;
  end

  always @ (posedge clk_12M) begin
    if(clockCtrl_resetUnbuffered_regNext) begin
      _zz_7_ <= 1'b0;
    end else begin
      _zz_7_ <= (systemDebugger_1__io_mem_cmd_valid && system_cpu_cpu_debug_bus_cmd_ready);
    end
  end

  always @ (posedge clk_12M) begin
    _zz_8_ <= system_spiA_phy_ss[0];
    _zz_9_ <= system_spiA_phy_sclk_write[1];
    _zz_10_ <= system_spiA_phy_data_0_write[1];
    sB_IO_3__D_IN_1_regNext <= sB_IO_3__D_IN_1;
    _zz_11_ <= system_spiA_phy_data_1_write[1];
    sB_IO_4__D_IN_1_regNext <= sB_IO_4__D_IN_1;
    if((generator_cmd_valid && generator_cmd_ready))begin
      generator_cmd_payload_fragment_context_regNextWhen <= generator_cmd_payload_fragment_context;
    end
    if(system_bridge_cmd_ready)begin
      _zz_27_ <= system_bridge_cmd_payload_last;
      _zz_28_ <= system_bridge_cmd_payload_fragment_source;
      _zz_29_ <= system_bridge_cmd_payload_fragment_opcode;
      _zz_30_ <= system_bridge_cmd_payload_fragment_address;
      _zz_31_ <= system_bridge_cmd_payload_fragment_length;
      _zz_32_ <= system_bridge_cmd_payload_fragment_data;
      _zz_33_ <= system_bridge_cmd_payload_fragment_mask;
      _zz_34_ <= system_bridge_cmd_payload_fragment_context;
    end
  end

endmodule

