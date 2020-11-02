(require 'clang-format)
(require 'clang-format+)

(add-hook 'c-mode-common-hook #'clang-format+-mode)

(provide 'my_clang_format)
