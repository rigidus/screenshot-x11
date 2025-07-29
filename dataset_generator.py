#!/usr/bin/env python3
# dataset_generator.py

import os
import random
from PIL import Image, ImageDraw, ImageFont

potential_fonts = [
    "/usr/share/fonts/opentype/cantarell/Cantarell-Regular.otf",
    "/usr/share/fonts/opentype/cantarell/Cantarell-Light.otf",
    "/usr/share/fonts/opentype/cantarell/Cantarell-ExtraBold.otf",
    "/usr/share/fonts/opentype/cantarell/Cantarell-Bold.otf",
    "/usr/share/fonts/opentype/freefont/FreeMonoBoldOblique.otf",
    "/usr/share/fonts/opentype/freefont/FreeSerifBold.otf",
    "/usr/share/fonts/opentype/freefont/FreeSerifBoldItalic.otf",
    "/usr/share/fonts/opentype/freefont/FreeMonoBold.otf",
    "/usr/share/fonts/opentype/freefont/FreeSansBoldOblique.otf",
    "/usr/share/fonts/opentype/freefont/FreeSans.otf",
    "/usr/share/fonts/opentype/freefont/FreeMonoOblique.otf",
    "/usr/share/fonts/opentype/freefont/FreeSansBold.otf",
    "/usr/share/fonts/opentype/freefont/FreeSansOblique.otf",
    "/usr/share/fonts/opentype/freefont/FreeMono.otf",
    "/usr/share/fonts/opentype/freefont/FreeSerifItalic.otf",
    "/usr/share/fonts/opentype/freefont/FreeSerif.otf",
    "/usr/share/fonts/opentype/urw-base35/P052-BoldItalic.otf",
    "/usr/share/fonts/opentype/urw-base35/URWGothic-Book.otf",
    "/usr/share/fonts/opentype/urw-base35/URWBookman-DemiItalic.otf",
    "/usr/share/fonts/opentype/urw-base35/C059-Roman.otf",
    "/usr/share/fonts/opentype/urw-base35/NimbusRoman-Regular.otf",
    "/usr/share/fonts/opentype/urw-base35/NimbusMonoPS-Bold.otf",
    "/usr/share/fonts/opentype/urw-base35/NimbusRoman-Italic.otf",
    "/usr/share/fonts/opentype/urw-base35/NimbusSansNarrow-Oblique.otf",
    "/usr/share/fonts/opentype/urw-base35/P052-Roman.otf",
    "/usr/share/fonts/opentype/urw-base35/NimbusMonoPS-BoldItalic.otf",
    "/usr/share/fonts/opentype/urw-base35/URWGothic-DemiOblique.otf",
    "/usr/share/fonts/opentype/urw-base35/Z003-MediumItalic.otf",
    "/usr/share/fonts/opentype/urw-base35/C059-Italic.otf",
    "/usr/share/fonts/opentype/urw-base35/URWBookman-Demi.otf",
    "/usr/share/fonts/opentype/urw-base35/P052-Bold.otf",
    "/usr/share/fonts/opentype/urw-base35/NimbusSans-BoldItalic.otf",
    "/usr/share/fonts/opentype/urw-base35/NimbusSans-Italic.otf",
    "/usr/share/fonts/opentype/urw-base35/NimbusSans-Bold.otf",
    "/usr/share/fonts/opentype/urw-base35/NimbusSans-Regular.otf",
    "/usr/share/fonts/opentype/urw-base35/URWGothic-BookOblique.otf",
    "/usr/share/fonts/opentype/urw-base35/NimbusMonoPS-Italic.otf",
    "/usr/share/fonts/opentype/urw-base35/NimbusSansNarrow-Regular.otf",
    "/usr/share/fonts/opentype/urw-base35/URWBookman-Light.otf",
    "/usr/share/fonts/opentype/urw-base35/NimbusSansNarrow-BoldOblique.otf",
    "/usr/share/fonts/opentype/urw-base35/P052-Italic.otf",
    "/usr/share/fonts/opentype/urw-base35/NimbusRoman-Bold.otf",
    "/usr/share/fonts/opentype/urw-base35/URWBookman-LightItalic.otf",
    "/usr/share/fonts/opentype/urw-base35/C059-BdIta.otf",
    "/usr/share/fonts/opentype/urw-base35/NimbusMonoPS-Regular.otf",
    "/usr/share/fonts/opentype/urw-base35/NimbusRoman-BoldItalic.otf",
    "/usr/share/fonts/opentype/urw-base35/URWGothic-Demi.otf",
    "/usr/share/fonts/opentype/urw-base35/C059-Bold.otf",
    "/usr/share/fonts/opentype/urw-base35/NimbusSansNarrow-Bold.otf",
    "/usr/share/fonts/opentype/artemisia/GFSArtemisia.otf",
    "/usr/share/fonts/opentype/stix-word/STIX-Regular.otf",
    "/usr/share/fonts/opentype/stix-word/STIX-Italic.otf",
    "/usr/share/fonts/opentype/stix-word/STIX-Bold.otf",
    "/usr/share/fonts/opentype/stix-word/STIX-BoldItalic.otf",
    "/usr/share/fonts/opentype/stix/STIXGeneral-Italic.otf",
    "/usr/share/fonts/opentype/stix/STIXGeneral-Bold.otf",
    "/usr/share/fonts/opentype/stix/STIXGeneral-BoldItalic.otf",
    "/usr/share/fonts/opentype/stix/STIXGeneral-Regular.otf",
    "/usr/share/fonts/opentype/didot/GFSDidotItalic.otf",
    "/usr/share/fonts/opentype/didot/GFSDidotBoldItalic.otf",
    "/usr/share/fonts/opentype/didot/GFSDidotBold.otf",
    "/usr/share/fonts/opentype/didot/GFSDidot.otf",
    "/usr/share/fonts/opentype/neohellenic/GFSNeohellenic.otf",
    "/usr/share/fonts/opentype/neohellenic/GFSNeohellenicIt.otf",
    "/usr/share/fonts/opentype/neohellenic/GFSNeohellenicBoldIt.otf",
    "/usr/share/fonts/opentype/ebgaramond/EBGaramond08-Regular.otf",
    "/usr/share/fonts/opentype/ebgaramond/EBGaramondSC08-Regular.otf",
    "/usr/share/fonts/opentype/ebgaramond/EBGaramond12-AllSC.otf",
    "/usr/share/fonts/opentype/ebgaramond/EBGaramond12-Regular.otf",
    "/usr/share/fonts/opentype/ebgaramond/EBGaramond12-Italic.otf",
    "/usr/share/fonts/opentype/ebgaramond/EBGaramondSC12-Regular.otf",
    "/usr/share/fonts/opentype/linux-libertine/LinLibertine_RZ.otf",
    "/usr/share/fonts/opentype/linux-libertine/LinLibertine_DR.otf",
    "/usr/share/fonts/opentype/linux-libertine/LinBiolinum_R.otf",
    "/usr/share/fonts/opentype/linux-libertine/LinLibertine_R.otf",
    "/usr/share/fonts/opentype/linux-libertine/LinLibertine_I.otf",
    "/usr/share/fonts/opentype/linux-libertine/LinBiolinum_RI.otf",
    "/usr/share/fonts/opentype/linux-libertine/LinLibertine_RZI.otf",
    "/usr/share/fonts/opentype/linux-libertine/LinBiolinum_RB.otf",
    "/usr/share/fonts/opentype/linux-libertine/LinLibertine_RI.otf",
    "/usr/share/fonts/opentype/linux-libertine/LinLibertine_RB.otf",
    "/usr/share/fonts/truetype/roboto/unhinted/RobotoCondensed-Bold.ttf",
    "/usr/share/fonts/truetype/roboto/unhinted/RobotoCondensed-Regular.ttf",
    "/usr/share/fonts/truetype/roboto/unhinted/RobotoCondensed-BoldItalic.ttf",
    "/usr/share/fonts/truetype/roboto/unhinted/RobotoCondensed-Medium.ttf",
    "/usr/share/fonts/truetype/roboto/unhinted/RobotoCondensed-Light.ttf",
    "/usr/share/fonts/truetype/roboto/unhinted/RobotoCondensed-LightItalic.ttf",
    "/usr/share/fonts/truetype/roboto/unhinted/RobotoTTF/Roboto-Italic.ttf",
    "/usr/share/fonts/truetype/roboto/unhinted/RobotoTTF/Roboto-Light.ttf",
    "/usr/share/fonts/truetype/roboto/unhinted/RobotoTTF/Roboto-Bold.ttf",
    "/usr/share/fonts/truetype/roboto/unhinted/RobotoTTF/Roboto-Thin.ttf",
    "/usr/share/fonts/truetype/roboto/unhinted/RobotoTTF/Roboto-Black.ttf",
    "/usr/share/fonts/truetype/roboto/unhinted/RobotoTTF/Roboto-LightItalic.ttf",
    "/usr/share/fonts/truetype/roboto/unhinted/RobotoTTF/Roboto-Medium.ttf",
    "/usr/share/fonts/truetype/roboto/unhinted/RobotoTTF/Roboto-Regular.ttf",
    "/usr/share/fonts/truetype/roboto/unhinted/RobotoTTF/Roboto-MediumItalic.ttf",
    "/usr/share/fonts/truetype/roboto/unhinted/RobotoTTF/Roboto-BoldItalic.ttf",
    "/usr/share/fonts/truetype/roboto/unhinted/RobotoTTF/Roboto-BlackItalic.ttf",
    "/usr/share/fonts/truetype/roboto/unhinted/RobotoCondensed-Italic.ttf",
    "/usr/share/fonts/truetype/roboto/unhinted/RobotoCondensed-MediumItalic.ttf",
    "/usr/share/fonts/truetype/crosextra/Carlito-BoldItalic.ttf",
    "/usr/share/fonts/truetype/crosextra/Carlito-Regular.ttf",
    "/usr/share/fonts/truetype/crosextra/Carlito-Italic.ttf",
    "/usr/share/fonts/truetype/crosextra/Carlito-Bold.ttf",
    "/usr/share/fonts/truetype/croscore/Arimo-BoldItalic.ttf",
    "/usr/share/fonts/truetype/croscore/Cousine-Italic.ttf",
    "/usr/share/fonts/truetype/croscore/Tinos-Regular.ttf",
    "/usr/share/fonts/truetype/croscore/Tinos-BoldItalic.ttf",
    "/usr/share/fonts/truetype/croscore/Arimo-Italic.ttf",
    "/usr/share/fonts/truetype/croscore/Cousine-Regular.ttf",
    "/usr/share/fonts/truetype/croscore/Arimo-Regular.ttf",
    "/usr/share/fonts/truetype/croscore/Arimo-Bold.ttf",
    "/usr/share/fonts/truetype/croscore/Tinos-Bold.ttf",
    "/usr/share/fonts/truetype/croscore/Cousine-Bold.ttf",
    "/usr/share/fonts/truetype/croscore/Tinos-Italic.ttf",
    "/usr/share/fonts/truetype/croscore/Cousine-BoldItalic.ttf",
    "/usr/share/fonts/truetype/gentiumplus/GentiumPlus-Regular.ttf",
    "/usr/share/fonts/truetype/gentiumplus/GentiumBookPlus-BoldItalic.ttf",
    "/usr/share/fonts/truetype/gentiumplus/GentiumBookPlus-Regular.ttf",
    "/usr/share/fonts/truetype/gentiumplus/GentiumPlus-Italic.ttf",
    "/usr/share/fonts/truetype/gentiumplus/GentiumBookPlus-Italic.ttf",
    "/usr/share/fonts/truetype/gentiumplus/GentiumBookPlus-Bold.ttf",
    "/usr/share/fonts/truetype/liberation/LiberationSerif-Regular.ttf",
    "/usr/share/fonts/truetype/liberation/LiberationMono-Regular.ttf",
    "/usr/share/fonts/truetype/liberation/LiberationSansNarrow-BoldItalic.ttf",
    "/usr/share/fonts/truetype/liberation/LiberationSans-BoldItalic.ttf",
    "/usr/share/fonts/truetype/liberation/LiberationSerif-Bold.ttf",
    "/usr/share/fonts/truetype/liberation/LiberationSans-Bold.ttf",
    "/usr/share/fonts/truetype/liberation/LiberationSans-Italic.ttf",
    "/usr/share/fonts/truetype/liberation/LiberationSansNarrow-Bold.ttf",
    "/usr/share/fonts/truetype/liberation/LiberationMono-BoldItalic.ttf",
    "/usr/share/fonts/truetype/liberation/LiberationSansNarrow-Italic.ttf",
    "/usr/share/fonts/truetype/liberation/LiberationSansNarrow-Regular.ttf",
    "/usr/share/fonts/truetype/liberation/LiberationMono-Italic.ttf",
    "/usr/share/fonts/truetype/liberation/LiberationSans-Regular.ttf",
    "/usr/share/fonts/truetype/liberation/LiberationSerif-Italic.ttf",
    "/usr/share/fonts/truetype/liberation/LiberationSerif-BoldItalic.ttf",
    "/usr/share/fonts/truetype/liberation/LiberationMono-Bold.ttf",
    "/usr/share/fonts/truetype/adf/BerenisADFPro-Italic.otf",
    "/usr/share/fonts/truetype/adf/BerenisADFPro-Regular.otf",
    "/usr/share/fonts/truetype/adf/BerenisADFPro-BoldItalic.otf",
    "/usr/share/fonts/truetype/adf/BerenisADFPro-Bold.otf",
    "/usr/share/fonts/truetype/adf/BerenisADFProMath-Regular.otf",
    "/usr/share/fonts/truetype/freefont/FreeSerif.ttf",
    "/usr/share/fonts/truetype/freefont/FreeSerifBold.ttf",
    "/usr/share/fonts/truetype/freefont/FreeMono.ttf",
    "/usr/share/fonts/truetype/freefont/FreeSansOblique.ttf",
    "/usr/share/fonts/truetype/freefont/FreeSansBold.ttf",
    "/usr/share/fonts/truetype/freefont/FreeMonoBold.ttf",
    "/usr/share/fonts/truetype/freefont/FreeSansBoldOblique.ttf",
    "/usr/share/fonts/truetype/freefont/FreeSerifBoldItalic.ttf",
    "/usr/share/fonts/truetype/freefont/FreeSerifItalic.ttf",
    "/usr/share/fonts/truetype/freefont/FreeMonoOblique.ttf",
    "/usr/share/fonts/truetype/freefont/FreeSans.ttf",
    "/usr/share/fonts/truetype/freefont/FreeMonoBoldOblique.ttf",
    "/usr/share/fonts/truetype/comfortaa/Comfortaa-Bold.ttf",
    "/usr/share/fonts/truetype/comfortaa/Comfortaa-Light.ttf",
    "/usr/share/fonts/truetype/comfortaa/Comfortaa-Regular.ttf",
    "/usr/share/fonts/truetype/noto/NotoSerif-Bold.ttf",
    "/usr/share/fonts/truetype/noto/NotoSerif-Italic.ttf",
    "/usr/share/fonts/truetype/noto/NotoSansDisplay-Regular.ttf",
    "/usr/share/fonts/truetype/noto/NotoSerif-BoldItalic.ttf",
    "/usr/share/fonts/truetype/noto/NotoSansOldPermic-Regular.ttf",
    "/usr/share/fonts/truetype/noto/NotoSans-BoldItalic.ttf",
    "/usr/share/fonts/truetype/noto/NotoSans-Italic.ttf",
    "/usr/share/fonts/truetype/noto/NotoSerifDisplay-Regular.ttf",
    "/usr/share/fonts/truetype/noto/NotoSerifDisplay-Italic.ttf",
    "/usr/share/fonts/truetype/noto/NotoSans-Bold.ttf",
    "/usr/share/fonts/truetype/noto/NotoSerifDisplay-BoldItalic.ttf",
    "/usr/share/fonts/truetype/noto/NotoSans-Regular.ttf",
    "/usr/share/fonts/truetype/noto/NotoSerif-Regular.ttf",
    "/usr/share/fonts/truetype/noto/NotoSerifDisplay-Bold.ttf",
    "/usr/share/fonts/truetype/noto/NotoSansGlagolitic-Regular.ttf",
    "/usr/share/fonts/truetype/noto/NotoSansDisplay-Bold.ttf",
    "/usr/share/fonts/truetype/noto/NotoSansDisplay-Italic.ttf",
    "/usr/share/fonts/truetype/noto/NotoSansMono-Bold.ttf",
    "/usr/share/fonts/truetype/noto/NotoSansMono-Regular.ttf",
    "/usr/share/fonts/truetype/noto/NotoMono-Regular.ttf",
    "/usr/share/fonts/truetype/noto/NotoSansDisplay-BoldItalic.ttf",
    "/usr/share/fonts/truetype/ubuntu/UbuntuMono-B.ttf",
    "/usr/share/fonts/truetype/ubuntu/UbuntuMono-R.ttf",
    "/usr/share/fonts/truetype/ubuntu/Ubuntu-M.ttf",
    "/usr/share/fonts/truetype/ubuntu/Ubuntu-Th.ttf",
    "/usr/share/fonts/truetype/ubuntu/Ubuntu-LI.ttf",
    "/usr/share/fonts/truetype/ubuntu/Ubuntu-B.ttf",
    "/usr/share/fonts/truetype/ubuntu/Ubuntu-R.ttf",
    "/usr/share/fonts/truetype/ubuntu/UbuntuMono-RI.ttf",
    "/usr/share/fonts/truetype/ubuntu/Ubuntu-BI.ttf",
    "/usr/share/fonts/truetype/ubuntu/Ubuntu-RI.ttf",
    "/usr/share/fonts/truetype/ubuntu/Ubuntu-C.ttf",
    "/usr/share/fonts/truetype/ubuntu/Ubuntu-L.ttf",
    "/usr/share/fonts/truetype/ubuntu/UbuntuMono-BI.ttf",
    "/usr/share/fonts/truetype/ubuntu/Ubuntu-MI.ttf",
    "/usr/share/fonts/truetype/lato/Lato-Heavy.ttf",
    "/usr/share/fonts/truetype/lato/Lato-Light.ttf",
    "/usr/share/fonts/truetype/lato/Lato-Medium.ttf",
    "/usr/share/fonts/truetype/lato/Lato-Regular.ttf",
    "/usr/share/fonts/truetype/lato/Lato-SemiboldItalic.ttf",
    "/usr/share/fonts/truetype/lato/Lato-BlackItalic.ttf",
    "/usr/share/fonts/truetype/lato/Lato-MediumItalic.ttf",
    "/usr/share/fonts/truetype/lato/Lato-Thin.ttf",
    "/usr/share/fonts/truetype/lato/Lato-HeavyItalic.ttf",
    "/usr/share/fonts/truetype/lato/Lato-Semibold.ttf",
    "/usr/share/fonts/truetype/lato/Lato-LightItalic.ttf",
    "/usr/share/fonts/truetype/lato/Lato-BoldItalic.ttf",
    "/usr/share/fonts/truetype/lato/Lato-ThinItalic.ttf",
    "/usr/share/fonts/truetype/lato/Lato-Black.ttf",
    "/usr/share/fonts/truetype/lato/Lato-Bold.ttf",
    "/usr/share/fonts/truetype/lato/Lato-Italic.ttf",
    "/usr/share/fonts/truetype/gentium/Gentium-R.ttf",
    "/usr/share/fonts/truetype/gentium/GentiumAlt-R.ttf",
    "/usr/share/fonts/truetype/charis/CharisSIL-Regular.ttf",
    "/usr/share/fonts/truetype/dejavu/DejaVuSansCondensed.ttf",
    "/usr/share/fonts/truetype/dejavu/DejaVuSansCondensed-Oblique.ttf",
    "/usr/share/fonts/truetype/dejavu/DejaVuSerif.ttf",
    "/usr/share/fonts/truetype/dejavu/DejaVuSansMono-Bold.ttf",
    "/usr/share/fonts/truetype/dejavu/DejaVuSans-Oblique.ttf",
    "/usr/share/fonts/truetype/dejavu/DejaVuSerif-Bold.ttf",
    "/usr/share/fonts/truetype/dejavu/DejaVuSans-BoldOblique.ttf",
    "/usr/share/fonts/truetype/dejavu/DejaVuSerifCondensed.ttf",
    "/usr/share/fonts/truetype/dejavu/DejaVuSerifCondensed-Italic.ttf",
    "/usr/share/fonts/truetype/dejavu/DejaVuSerifCondensed-Bold.ttf",
    "/usr/share/fonts/truetype/dejavu/DejaVuSerif-BoldItalic.ttf",
    "/usr/share/fonts/truetype/dejavu/DejaVuSerif-Italic.ttf",
    "/usr/share/fonts/truetype/dejavu/DejaVuSans-ExtraLight.ttf",
    "/usr/share/fonts/truetype/dejavu/DejaVuSansMono-Oblique.ttf",
    "/usr/share/fonts/truetype/dejavu/DejaVuSerifCondensed-BoldItalic.ttf",
    "/usr/share/fonts/truetype/dejavu/DejaVuSansCondensed-BoldOblique.ttf",
    "/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf",
    "/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf",
    "/usr/share/fonts/truetype/dejavu/DejaVuSansMono-BoldOblique.ttf",
    "/usr/share/fonts/truetype/dejavu/DejaVuSansMono.ttf",
    "/usr/share/fonts/truetype/dejavu/DejaVuSansCondensed-Bold.ttf",
    "/usr/share/fonts/truetype/liberation2/LiberationSerif-Regular.ttf",
    "/usr/share/fonts/truetype/liberation2/LiberationMono-Regular.ttf",
    "/usr/share/fonts/truetype/liberation2/LiberationSans-BoldItalic.ttf",
    "/usr/share/fonts/truetype/liberation2/LiberationSerif-Bold.ttf",
    "/usr/share/fonts/truetype/liberation2/LiberationSans-Bold.ttf",
    "/usr/share/fonts/truetype/liberation2/LiberationSans-Italic.ttf",
    "/usr/share/fonts/truetype/liberation2/LiberationMono-BoldItalic.ttf",
    "/usr/share/fonts/truetype/liberation2/LiberationMono-Italic.ttf",
    "/usr/share/fonts/truetype/liberation2/LiberationSans-Regular.ttf",
    "/usr/share/fonts/truetype/liberation2/LiberationSerif-Italic.ttf",
    "/usr/share/fonts/truetype/liberation2/LiberationSerif-BoldItalic.ttf",
    "/usr/share/fonts/truetype/liberation2/LiberationMono-Bold.ttf",
    "/usr/share/fonts/truetype/ebgaramond/EBGaramond08-Regular.ttf",
    "/usr/share/fonts/truetype/ebgaramond/EBGaramond12-Italic.ttf",
    "/usr/share/fonts/truetype/ebgaramond/EBGaramondSC08-Regular.ttf",
    "/usr/share/fonts/truetype/ebgaramond/EBGaramond12-Regular.ttf",
    "/usr/share/fonts/truetype/ebgaramond/EBGaramond12-AllSC.ttf",
    "/usr/share/fonts/truetype/ebgaramond/EBGaramondSC12-Regular.ttf",
    "/usr/share/fonts/truetype/msttcorefonts/Georgia_Bold_Italic.ttf",
    "/usr/share/fonts/truetype/msttcorefonts/Verdana_Italic.ttf",
    "/usr/share/fonts/truetype/msttcorefonts/Trebuchet_MS_Italic.ttf",
    "/usr/share/fonts/truetype/msttcorefonts/Georgia.ttf",
    "/usr/share/fonts/truetype/msttcorefonts/Times_New_Roman_Italic.ttf",
    "/usr/share/fonts/truetype/msttcorefonts/Trebuchet_MS.ttf",
    "/usr/share/fonts/truetype/msttcorefonts/Arial_Black.ttf",
    "/usr/share/fonts/truetype/msttcorefonts/Impact.ttf",
    "/usr/share/fonts/truetype/msttcorefonts/Verdana.ttf",
    "/usr/share/fonts/truetype/msttcorefonts/Verdana_Bold.ttf",
    "/usr/share/fonts/truetype/msttcorefonts/Courier_New_Italic.ttf",
    "/usr/share/fonts/truetype/msttcorefonts/Verdana_Bold_Italic.ttf",
    "/usr/share/fonts/truetype/msttcorefonts/Trebuchet_MS_Bold.ttf",
    "/usr/share/fonts/truetype/msttcorefonts/Arial_Bold.ttf",
    "/usr/share/fonts/truetype/msttcorefonts/Arial_Bold_Italic.ttf",
    "/usr/share/fonts/truetype/msttcorefonts/Georgia_Bold.ttf",
    "/usr/share/fonts/truetype/msttcorefonts/Trebuchet_MS_Bold_Italic.ttf",
    "/usr/share/fonts/truetype/msttcorefonts/Times_New_Roman.ttf",
    "/usr/share/fonts/truetype/msttcorefonts/Courier_New_Bold_Italic.ttf",
    "/usr/share/fonts/truetype/msttcorefonts/Times_New_Roman_Bold.ttf",
    "/usr/share/fonts/truetype/msttcorefonts/Andale_Mono.ttf",
    "/usr/share/fonts/truetype/msttcorefonts/Arial_Italic.ttf",
    "/usr/share/fonts/truetype/msttcorefonts/Times_New_Roman_Bold_Italic.ttf",
    "/usr/share/fonts/truetype/msttcorefonts/Courier_New_Bold.ttf",
    "/usr/share/fonts/truetype/msttcorefonts/Arial.ttf",
    "/usr/share/fonts/truetype/msttcorefonts/Courier_New.ttf",
    "/usr/share/fonts/truetype/msttcorefonts/Comic_Sans_MS_Bold.ttf",
    "/usr/share/fonts/truetype/msttcorefonts/Georgia_Italic.ttf",
    "/usr/share/fonts/truetype/open-sans/OpenSans-Light.ttf",
    "/usr/share/fonts/truetype/open-sans/OpenSans-Semibold.ttf",
    "/usr/share/fonts/truetype/open-sans/OpenSans-Regular.ttf",
    "/usr/share/fonts/truetype/open-sans/OpenSans-CondLight.ttf",
    "/usr/share/fonts/truetype/open-sans/OpenSans-CondLightItalic.ttf",
    "/usr/share/fonts/truetype/open-sans/OpenSans-CondBold.ttf",
    "/usr/share/fonts/truetype/open-sans/OpenSans-Bold.ttf",
    "/usr/share/fonts/truetype/open-sans/OpenSans-LightItalic.ttf",
    "/usr/share/fonts/truetype/open-sans/OpenSans-Italic.ttf",
    "/usr/share/fonts/truetype/open-sans/OpenSans-BoldItalic.ttf",
    "/usr/share/fonts/truetype/open-sans/OpenSans-SemiboldItalic.ttf",
    "/usr/share/fonts/truetype/open-sans/OpenSans-ExtraBold.ttf",
    "/usr/share/fonts/truetype/open-sans/OpenSans-ExtraBoldItalic.ttf",
    "/usr/share/fonts/truetype/gentiumplus-compact/GentiumPlusCompact-R.ttf",
    "/usr/share/fonts/truetype/gentiumplus-compact/GentiumPlusCompact-I.ttf",
    "/usr/share/fonts/fonts-go/Go-Bold.ttf",
    "/usr/share/fonts/fonts-go/Go-Mono.ttf",
    "/usr/share/fonts/fonts-go/Go-Medium-Italic.ttf",
    "/usr/share/fonts/fonts-go/Go-Mono-Bold-Italic.ttf",
    "/usr/share/fonts/fonts-go/Go-Regular.ttf",
    "/usr/share/fonts/fonts-go/Go-Mono-Bold.ttf",
    "/usr/share/fonts/fonts-go/Go-Italic.ttf",
    "/usr/share/fonts/fonts-go/Go-Smallcaps.ttf",
    "/usr/share/fonts/fonts-go/Go-Bold-Italic.ttf",
    "/usr/share/fonts/fonts-go/Go-Smallcaps-Italic.ttf",
    "/usr/share/fonts/fonts-go/Go-Medium.ttf",
    "/usr/share/fonts/fonts-go/Go-Mono-Italic.ttf",
]

potential_fonts = list(set(potential_fonts))


# Проверяем, какие шрифты доступны
available_fonts = []
for fp in potential_fonts:
    try:
        ImageFont.truetype(fp, size=32)
        available_fonts.append(fp)
    except (IOError, OSError):
        print(f"Font not found or cannot be loaded: {fp}, skipping.")

if not available_fonts:
    raise RuntimeError("No valid fonts found. Please install at least one TTF/OTF font from the list.")

# Группировка символов по классам
char_groups = {}
def add_group(key, chars):
    char_groups[key] = chars

# Цифры: каждая цифра свой класс
for d in "0123456789":
    add_group(d, [d])

# Карта похожих символов латиницы и кириллицы — объединяем в один класс
similar_map = {
    'A': ['A','А'], 'B':['B','В'], 'C':['C','С'], 'E':['E','Е'],
    'H':['H','Н'], 'K':['K','К'], 'M':['M','М'], 'O':['O','О'],
    'P':['P','Р'], 'T':['T','Т'], 'X':['X','Х'], 'Y':['Y','У'],
    'a':['a','а'], 'e':['e','е'], 'o':['o','о'], 'c':['c','с'],
    'p':['p','р'], 'x':['x','х'], 'y':['y','у'],
}
for key, chars in similar_map.items():
    add_group(key, chars)

# # Остальные латинские и русские буквы, не попавшие в similar_map
latin_upper = list("ABCDEFGHIJKLMNOPQRSTUVWXYZ")
latin_lower = list("abcdefghijklmnopqrstuvwxyz")
rus_upper   = [chr(c) for c in range(ord("А"), ord("Я")+1)]
rus_lower   = [chr(c) for c in range(ord("а"), ord("я")+1)]
for ch in latin_upper + latin_lower + rus_upper + rus_lower:
    # пропускаем те, что уже в similar_map
    if not any(ch in group for group in similar_map.values()):
        add_group(ch, [ch])



# Пунктуация: каждый символ отдельный класс
punct_classes = {
    'period': ['.'],
    'comma': [','],
    'semicolon': [';'],
    'colon': [':'],
    'exclamation': ['!'],
    'question': ['?'],
    'lparen': ['('],
    'rparen': [')'],
    'lbracket': ['['],
    'rbracket': [']'],
    'lbrace': ['{'],
    'rbrace': ['}'],
    'doublequote': ['"'],
    'quote': ["'"],
    'leftanglequote': ['«'],
    'rightanglequote': ['»'],
}
for key, chars in punct_classes.items():
    add_group(key, chars)

# Стрелки: каждый направленный символ отдельный класс
arrow_map = {
    'leftarrow': ['←'],
    'rightarrow': ['→'],
    'uparrow': ['↑'],
    'downarrow': ['↓'],
}
for key, chars in arrow_map.items():
    add_group(key, chars)

class_keys = list(char_groups.keys())


# Параметры генерации
samples_per_class       = 8000   # общее число примеров на класс
min_samples_per_font    = 30     # минимум на шрифт/класс
val_ratio               = 0.1    # доля валидационной выборки
canvas_size             = 48
crop_size               = 32
# Порог чёрных пикселей: по умолчанию и для каждого класса
default_min_black_pixels = 1

min_black_pixels_map     = {
    # Пример индивидуальных порогов:
    'A': 10,
    '0': 9,
    '.': 1,
    ',': 2,
    ':': 2,
}


# Функция генерации одного изображения
# Возвращает объект PIL.Image формата «1» (чёрно-белый), или None, если:
# символ выходит за границы обрезки,
# либо слишком мало чёрных пикселей (слишком тонкий/маленький).
def make_image(ch, font_path):

    # 1. Создаём белый холст 48×48
    img = Image.new("L", (canvas_size, canvas_size), color=255)
    draw = ImageDraw.Draw(img)

    # 2. Выбираем масштаб
    scale = random.uniform(0.3, 1.9)

    # загружаем шрифт
    font = ImageFont.truetype(font_path, int(32 * scale))

    # 3. Вычисляем bbox текста, случайно смещаем
    bbox = draw.textbbox((0, 0), ch, font=font)
    w, h = bbox[2] - bbox[0], bbox[3] - bbox[1]

    # 4. Проверяем, поместится ли в центральную область 32×32
    dx = random.randint(-5, 5)
    dy = random.randint(-5, 5)

    x = (canvas_size - w) // 2 + dx
    y = (canvas_size - h) // 2 + dy

    cx0 = (canvas_size - crop_size) // 2
    cy0 = cx0
    cx1, cy1 = cx0 + crop_size, cy0 + crop_size
    x0, y0 = x + bbox[0], y + bbox[1]
    x1, y1 = x + bbox[2], y + bbox[3]
    if x0 < cx0 or y0 < cy0 or x1 > cx1 or y1 > cy1:
        return None

    # 5. Рисуем текст чёрным, обрезаем до 32×32
    draw.text((x, y), ch, fill=0, font=font)
    cropped = img.crop((cx0, cy0, cx1, cy1))

    # 6. Бинаризация
    bw = cropped.point(lambda p: 255 if p > 128 else 0, mode='L')
    binary = bw.convert('1')

    # 7. Отбраковка по минимальному числу чёрных пикселей
    black_count = list(binary.getdata()).count(0)
    threshold = min_black_pixels_map.get(ch, default_min_black_pixels)
    if black_count < threshold:
        return None

    return binary


# Основная функция: генерируем по каждому символу в отдельную папку
def main():
    base_dir = "data"
    # 1. Создаём папки data/train/<класс> и data/val/<класс>
    for split in ("train", "val"):
        for key in class_keys:
            os.makedirs(os.path.join(base_dir, split, key), exist_ok=True)

    # 2. Считаем, сколько генерировать на каждый шрифт
    n_fonts = len(available_fonts)
    per_font = max(samples_per_class // n_fonts, min_samples_per_font)

    # 3. Для каждого класса и каждого шрифта генерируем изображения
    for key in class_keys:
        for font_idx, font_path in enumerate(available_fonts):
            generated = 0
            attempts = 0
            max_attempts = (per_font - generated) * 10
            while generated < per_font and attempts < max_attempts:
                attempts += 1
                # выбираем символ из группы (для похожих lat/rus - случайно)
                ch = random.choice(char_groups[key])
                img = make_image(ch, font_path)  # теперь с рандомом внутри
                if img is None:
                    continue
                # СОХРАНЕНИЕ
                # 90% — в train, 10% — в val
                split = "val" if random.random() < val_ratio else "train"
                filename = os.path.basename(font_path) # имя файла шрифта
                name_no_ext = os.path.splitext(filename)[0] # без расширения
                fname = f"{ord(ch)}_{font_idx:03d}_{name_no_ext}_{generated:04d}.bmp"
                out_path = os.path.join(base_dir, split, key, fname)
                img.save(out_path, format='BMP')
                #
                generated += 1

            if generated < per_font:
                print(f"Warning: for class '{key}' in font #{font_idx} '{font_path}' generated only {generated}/{per_font}")

    print("Генерация завершена!")

if __name__ == "__main__":
    main()
