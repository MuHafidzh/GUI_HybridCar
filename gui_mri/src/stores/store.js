import { defineStore } from "pinia";

export const useMesinStore = defineStore("alerts", {
  id: "store",
  state: () => {
    return {
      dataMesin: {
        busVoltage: 0, // Tambahkan properti busVoltage
      },
      utils: {},
    };
  },
  actions: {
    resetDataMesin() {
      this.dataMesin = {
        busVoltage: 0,
      };
    },
    updateBusVoltage(value) {
      this.dataMesin.busVoltage = value;
    },
  },
});