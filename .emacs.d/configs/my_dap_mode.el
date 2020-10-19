(require 'dap-ui)

;; Add languages
;; Python
(setq dap-python-executable "python3")
(require 'dap-python)

;; C/C++

;; lldb-vscode
;; Intructions to compile the executable:
;;  - clone the llvm-project repository locally
;;  - cd llvm-project
;;  - mkdir build
;;  - cd build
;;  - cmake ../llvm -DLLVM_ENABLE_PROJECTS="clang;libcxx;lldb"
;;  - make lldb lldb-server lldb-vscode

(setq dap-lldb-debug-program '("/home/angel/src/llvm-project/build/bin/lldb-vscode"))
(require 'dap-lldb)

;; (require 'dap-gdb-lldb)

(setq dap-print-io 1)

(dap-auto-configure-mode 1)

(provide 'my_dap_mode)


;; Templates
;; (dap-register-debug-template
;;   "GDB::Run debug cortex"
;;   (list :type "gdb"
;;         :cwd nil
;;         :request "launch"
;;         :target "/home/angel/src/pacflyer_PX4/PX4/build/px4_fmu-v5_default/px4_fmu-v5_default.elf"
;; 	:dap-server-path `("/home/angel/src/stlink/build/Release/bin/st-util")
;; 	:port ":4242"
;;         :name "LLDB::Run"))

;; (dap-register-debug-template
;;   "GDB::Run debug cortex server"
;;   (list :type "gdbserver"
;;         :cwd nil
;;         :request "launch"
;;         :target "localhost:4242"
;; 	:dap-server-path `("/home/angel/src/stlink/build/Release/bin/st-util")
;;         :name "LLDB::Run server"))
