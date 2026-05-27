%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% CORESENSE report LaTeX class file
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

\NeedsTeXFormat{LaTeX2e}[1995/12/01]
\ProvidesClass{csreport}[2023/01/18 v1.1 CORESENSE LaTeX report class]

\usepackage[xetex,table,dvipsnames,hyperref]{xcolor}

% Green as default
%\definecolor{CSCover}{HTML}{006A63}
%\definecolor{CSCoverText}{HTML}{FFFFFF}
%\newcommand{\@background}{}
%\newcommand{\@coverlogo}{coresense-logo-reverse-reduced.png}

% Iris as default
\definecolor{CSCover}{HTML}{512977}
\definecolor{CSCoverText}{HTML}{FFFFFF}
\newcommand{\@background}{\includegraphics[scale=1.2]{CSirisbackground.png}}
\newcommand{\@coverlogo}{coresense-logo-reverse-reduced.png}

\newcommand{\@delno}{}
\newcommand{\delno}[1]{\renewcommand{\@delno}{#1\ }}
\newcommand{\@dissemination}{} % PU - Public, SEN - Sensitive
\newcommand{\dissemination}[1]{\renewcommand{\@dissemination}{#1}}

\DeclareOption{iris}{
  \definecolor{CSCover}{HTML}{512977}
  \definecolor{CSCoverText}{HTML}{FFFFFF}
  \renewcommand{\@coverlogo}{coresense-logo-reverse-reduced.png}
  \renewcommand{\@background}{\includegraphics[scale=1.2]{CSirisbackground.png}}
}
  
\DeclareOption{gradient}{
  \definecolor{CSCover}{HTML}{EAB82D}
  \definecolor{CSCoverText}{HTML}{000000}
  \renewcommand{\@coverlogo}{coresense-logo-reduced.png}
  \renewcommand{\@background}{\includegraphics[scale=1.2]{CSgradientbackground.png}}
}
  
\DeclareOption{green}{
  \definecolor{CSCover}{HTML}{006A63}
  \definecolor{CSCoverText}{HTML}{FFFFFF}
  \renewcommand{\@coverlogo}{coresense-logo-reverse-reduced.png}
  \renewcommand{\@background}{}
}

\DeclareOption{maroon}{
  \definecolor{CSCover}{HTML}{512977}
  \definecolor{CSCoverText}{HTML}{FFFFFF}
  \renewcommand{\@coverlogo}{coresense-logo-reverse-reduced.png}
  \renewcommand{\@background}{}
}

\DeclareOption{red}{
  \definecolor{CSCover}{HTML}{BF002A}
  \definecolor{CSCoverText}{HTML}{FFFFFF}
  \renewcommand{\@coverlogo}{coresense-logo-reverse-reduced.png}
  \renewcommand{\@background}{}
  \dissemination{Sensitive}
}

\DeclareOption{sensitive}{
  \definecolor{CSCover}{HTML}{BF002A}
  \definecolor{CSCoverText}{HTML}{FFFFFF}
  \renewcommand{\@coverlogo}{coresense-logo-reverse-reduced.png}
  \renewcommand{\@background}{}
  \dissemination{Sensitive}
}

\DeclareOption{orange}{
  \definecolor{CSCover}{HTML}{EAB82D}
  \definecolor{CSCoverText}{HTML}{000000}
  \renewcommand{\@coverlogo}{coresense-logo-reduced.png}
  \renewcommand{\@background}{}
}

\DeclareOption{white}{
  \definecolor{CSCover}{HTML}{FFFFFF}
  \definecolor{CSCoverText}{HTML}{000000}
  \renewcommand{\@coverlogo}{coresense-logo-reduced.png}
  \renewcommand{\@background}{}
}

\DeclareOption{black}{
  \definecolor{CSCover}{HTML}{000000}
  \definecolor{CSCoverText}{HTML}{FFFFFF}
  \renewcommand{\@coverlogo}{coresense-logo-reverse-reduced.png}
  \renewcommand{\@background}{}
}

\DeclareOption{deliverable}{
  \definecolor{CSCover}{HTML}{EAB82D}
  \definecolor{CSCoverText}{HTML}{000000}
  \renewcommand{\@coverlogo}{coresense-logo-reduced.png}
  \renewcommand{\@background}{}
  \delno{DX.Y}
  \dissemination{Sensitive}
}


  
\DeclareOption*{\PassOptionsToClass{\CurrentOption}{article}}
\ProcessOptions\relax

% Base LaTeX Class ---------------------------------------------------

\LoadClass[a4paper,11pt,twoside,onecolumn,final,openright]{report}

\usepackage{cscommon}

\usepackage{geometry}
\usepackage{lastpage}
\usepackage{watermark}
\usepackage{booktabs}
\usepackage{array}
\usepackage{ragged2e}
\usepackage{titlesec}

%-- Section formats -------------------------------------------------------------

\titleformat
{\chapter} % command
%[display] % shape
{\huge\headingfont} % format
{\color{CSGreen}\rule{6mm}{6mm}\color{CSRussianGray}\quad \thechapter} % label
{0.5ex} % sep
{
} % before-code
[
] % after-code

\titlespacing{\section}{0mm}{5mm}{0mm}
\titlespacing{\subsection}{0mm}{2mm}{0mm}
\titlespacing{\subsubsection}{0mm}{0mm}{0mm}

\newfontfamily\headingfont{Montserrat Bold} % Install ttf instead of otf
\newfontfamily\displayfont{Montserrat Black}

\titleformat*{\chapter}{\huge\headingfont}
\titleformat*{\section}{\Large\headingfont}
\titleformat*{\subsection}{\large\headingfont}
\titleformat*{\subsubsection}{\headingfont}

%\titlespacing*{\section}{0pt}{2ex plus 0.5ex minus .2ex}{1ex plus .2ex}
%\titlespacing*{\subsection}{0pt}{1.5ex plus 0.5ex minus .2ex}{1ex plus .2ex}

%-- Variables -------------------------------------------------------------



\newcommand{\@subtitle}{}
\newcommand{\subtitle}[1]{\renewcommand{\@subtitle}{#1}}

\newcommand{\@finalsubtitle}{\@subtitle}
\newcommand{\finalsubtitle}[1]{\renewcommand{\@finalsubtitle}{#1}}

\newcommand{\@shortauthor}{\@author}
\newcommand{\shortauthor}[1]{\renewcommand{\@shortauthor}{#1}}

\newcommand{\@finalauthor}{\@author}
\newcommand{\finalauthor}[1]{\renewcommand{\@finalauthor}{#1}}

\newcommand{\@shorttitle}{\@title}
\newcommand{\shorttitle}[1]{\renewcommand{\@shorttitle}{#1}}

\newcommand{\@finaltitle}{\@title}
\newcommand{\finaltitle}[1]{\renewcommand{\@finaltitle}{#1}}

\newcommand{\@covertitle}{\@title}
\newcommand{\covertitle}[1]{\renewcommand{\@covertitle}{#1}}

\newcommand{\@reference}{}
\newcommand{\reference}[1]{\renewcommand{\@reference}{#1}}

\newcommand{\@version}{0.0}
\newcommand{\version}[1]{\renewcommand{\@version}{#1}}

\newcommand{\@copyrightyear}{2022-2026}
\newcommand{\copyrightyear}[1]{\renewcommand{\@copyrightyear}{#1}}

\newcommand{\@status}{}
\newcommand{\status}[1]{\renewcommand{\@status}{#1}}

\newcommand{\@URL}{http://www.coresense.eu/doc/\@reference.pdf}
\newcommand{\URL}[1]{\renewcommand{\@URL}{#1}}

\newcommand{\@abstract}{}
\renewcommand{\abstract}[1]{\renewcommand{\@abstract}{#1}}

\newcommand{\@keywords}{}
\newcommand{\keywords}[1]{\renewcommand{\@keywords}{#1}}

\newcommand{\@acknowledgements}{}
\newcommand{\acknowledgements}[1]{\renewcommand{\@acknowledgements}{#1}}

\newcommand{\@logo}{coresense-logo-reduced.png}
\newcommand{\logo}[1]{\renewcommand{\@logo}{#1}}

%\newcommand{\@coverlogo}{coresense-logo-reverse-reduced.png} % Put at begining to process options
\newcommand{\coverlogo}[1]{\renewcommand{\@coverlogo}{#1}}

%\newcommand{\@background}{CSirisbackground.png} % Put at begining to process options
\newcommand{\background}[1]{\renewcommand{\@background}{\includegraphics[scale=1.2]{#1}}}


%-- Cover colors -------------------------------------------------------------

\newcommand\orangecover{\definecolor{CSCover}{HTML}{EAB82D}
                        \definecolor{CSCoverText}{HTML}{000000}
                        \coverlogo{coresense-logo-reduced.png}
                        \background{\renewcommand{\@background}{}}}
\newcommand\greencover{\definecolor{CSCover}{HTML}{006A63}
                       \definecolor{CSCoverText}{HTML}{FFFFFF}
                       \coverlogo{coresense-logo-reverse-reduced.png}
                        \background{\renewcommand{\@background}{}}}
\newcommand\redcover{\definecolor{CSCover}{HTML}{BF002A}
                     \definecolor{CSCoverText}{HTML}{FFFFFF}
                     \coverlogo{coresense-logo-reverse-reduced.png}
                        \background{\renewcommand{\@background}{}}}
\newcommand\marooncover{\definecolor{CSCover}{HTML}{512977}
                        \definecolor{CSCoverText}{HTML}{FFFFFF}
                        \coverlogo{coresense-logo-reverse-reduced.png}
                        \background{\renewcommand{\@background}{}}}

%-- Front cover -------------------------------------------------------------

\renewcommand\maketitle{
\newgeometry{top=20mm, left=20mm, bottom=20mm, right=20mm} 
\pagestyle{empty}

\hypersetup{urlcolor = CSCoverText}

\thiswatermark{\put(-100,-850){\@background} }

\parindent=0 true mm

\pagecolor{CSCover}

\fontsize{20}{20}\selectfont 
\textcolor{CSCoverText}{\href{\@URL}{\@reference-v\@version}}\\
\fontsize{15}{20}\selectfont 
\textcolor{CSCoverText}{\@date}\\
\fontsize{15}{20}\selectfont 
\textcolor{CSCoverText}{\@dissemination}
\vspace{5cm}

\parbox{17cm}{\raggedright\fontsize{40}{40}\selectfont 
\textcolor{CSCoverText}{\headingfont{\@title}}}
\vspace{2mm}

\parbox{17cm}{\raggedright\fontsize{20}{25}\selectfont  
\textcolor{CSCoverText}{\headingfont{\@subtitle}}}
\vspace{20mm}

\normalsize
\fontsize{20}{25}\selectfont \textcolor{CSCoverText}{\noindent\@author}
  
\vfill
\normalsize

\fontsize{20}{15}\selectfont 
\textcolor{CSCoverText}{\href{http://www.coresense.eu}{CORESENSE}}\\
\fontsize{15}{15}\selectfont 
\textcolor{CSCoverText}{Horizon Europe Project \#101070254}\\
\fontsize{11}{15}\selectfont 
\textcolor{CSCoverText}{HORIZON-CL4-2021-DIGITAL-EMERGING-01-11}
\vspace{3cm}

\begin{picture}(0,0)
\put(420,680){\euflag{0.055}}
\put(330,717){\fontsize{20}{20}\selectfont \textcolor{CSCoverText}{HORIZON}}
\put(342,698){\fontsize{20}{20}\selectfont \textcolor{CSCoverText}{EUROPE}}
\put(-10,0){\href{http://www.coresense.eu}{\includegraphics[height=3cm]{\@coverlogo}}}
\put(0,535){\fontsize{40}{40}\selectfont 
\textcolor{CSCoverText}{\href{\@URL}{\headingfont{\@delno}}}}
\end{picture}

\hypersetup{urlcolor = URLcolor}

\thispagestyle{empty}

\newpage
\nopagecolor
\pagecolor{white}%\afterpage{\nopagecolor}


\href{http://www.coresense.eu}{\includegraphics[height=2cm]{\@logo}}

\vspace{1cm}

{\small

The \textbf{CORESENSE Project} is a four year research project 
focused in the development of a \emph{technology of machine understanding} 
for the construction of dependable autonomous systems. It is a project funded 
by the European Union.

\begin{tabular}{ll}
Programme:& Horizon Europe\\
Project number:& 101070254\\
Project name:& CoreSense: A Hybrid Cognitive Architecture for Deep Understanding \\
Project acronym:& CORESENSE\\
Topic:& HORIZON-CL4-2021-DIGITAL-EMERGING-01-11\\
Type of action:& HORIZON Research and Innovation Actions\\
Granting authority:& European Commission-EU\\
Project start date:& 1 October 2022\\
Project end date:& 30 September 2026\\
Project duration:& 48 months\\
\end{tabular}

Most of CORESENSE results are open science but IPR regulations may still hold. 
Requests for permission to reproduce this document or to prepare derivative works 
of this document should be addressed to the CORESENSE Consortium IPR management.
     
Copyright $\copyright$ 2022-2026 by The CORESENSE Consortium.

UNIVERSIDAD POLITECNICA DE MADRID (UPM)\\
TECHNISCHE UNIVERSITEIT DELFT (TUD)\\
FRAUNHOFER GESELLSCHAFT ZUR FORDERUNG DER ANGEWANDTEN FORSCHUNG (FHG)\\
UNIVERSIDAD REY JUAN CARLOS (URJC)\\
PAL ROBOTICS SL (PAL)\\
IRISH MANUFACTURING RESEARCH COMPANY LIMITED BY GUARANTEE (IMR)\\
CZECH TECHNICAL UNIVERSITY IN PRAGE (CVUT)

\includegraphics[height=0.89cm]{UPM-logo.png}
\includegraphics[height=0.89cm]{TUD-logo.png} 
\includegraphics[height=0.89cm]{FHG-logo.png} 
\includegraphics[height=0.89cm]{URJC-logo.png} 
\includegraphics[height=0.89cm]{PAL-logo.png}
\includegraphics[height=0.89cm]{IMR-logo.png}
\includegraphics[height=0.89cm]{CVUT-logo.jpg}

For information about other CORESENSE products, reports or articles, please visit 
our Web site:

\href{http://www.coresense.eu}{\texttt{http://www.coresense.eu/}}


\vfill

NO WARRANTY --- Please read the fine print:

{\scriptsize
THIS CORESENSE MATERIAL IS FURNISHED ON AN ``AS-IS'' BASIS. THE CORESENSE CONSORTIUM 
NOR ANY OF THE INDIVIDUAL CORESENSE PARTNERS OR RESEARCHERS MAKE NO WARRANTIES OF 
ANY KIND, EITHER EXPRESSED OR IMPLIED, AS TO ANY MATTER INCLUDING, BUT NOT LIMITED 
TO, ACCURACY, COMPLETENESS, WARRANTY OF FITNESS FOR PURPOSE OR MERCHANTABILITY, 
EXCLUSIVITY, OR RESULTS OBTAINED FROM USE OF THE MATERIAL. CORESENSE DOES NOT MAKE 
ANY WARRANTY OF ANY KIND WITH RESPECT TO FREEDOM FROM PATENT, TRADEMARK, OR COPYRIGHT 
INFRINGEMENT.}\\

\parbox{60mm}{\href{http://www.horizoneurope.eu}{
\includegraphics[width=60mm]{EU_funded_en.jpg}}}
\parbox{11cm}{\scriptsize The CORESENSE project is funded by the EC Horizon Europe 
programme though grant HE \#101070254 inside the 
HORIZON-CL4-2021-DIGITAL-EMERGING-01-11 topic.
Views and opinions expressed in this document are however those of the author(s) 
only and do not necessarily reflect those of the European Union or the Horizon 
Europe Programme. Neither the European Union nor the granting authority can be held 
responsible for them.}
}
 
\cleardoublepage
\setnormalpage
}

%-- Final cover -------------------------------------------------------------

\newcommand{\finalcover}{

\cleardoublepage\newpage\newpage
\newgeometry{top=20mm, left=20mm, bottom=20mm, right=20mm} 
\pagestyle{empty}

\pagestyle{empty}
\pagecolor{CSCover}

\thiswatermark{\put(-100,-850){\@background}}

\vspace*{\stretch{2}}
\renewcommand{\arraystretch}{1.0}

\begin{table}[htp]\sffamily\color{CSCoverText}
\begin{tabular}{rl}
~     & \hspace{-1cm}\includegraphics[width=4cm]{\@coverlogo} \\[5mm]
\textit{Title}     & \parbox{14cm}{\Large\bfseries \@delno \@finaltitle} \\[2mm]
\textit{Subtitle}  & \parbox{14cm}{\@finalsubtitle}\\[1mm]
\textit{Author}    & \parbox{14cm}{\@finalauthor}\\[2mm]
\textit{Date}      & \@date\\
\textit{Reference} & \@reference\\
\textit{Version}   & \@version\ \@status\\[2mm]
\textit{URL}       & \@URL\\[4mm]
\copyright\ \@copyrightyear & {\large \href{http://www.coresense.eu}{
\color{CSCoverText} The CORESENSE Horizon Europe Project}}\\[2mm]
~  & {\Large\bfseries 
\href{http://www.coresense.eu}{\color{CSCoverText}coresense.eu}}\\[2mm]
\end{tabular}
\end{table}%

\begin{picture}(0,0)
\put(0,700){\euflag{0.04}}
\put(73,722){\fontsize{20}{20}\selectfont \textcolor{CSCoverText}{HORIZON}}
\put(73,702){\fontsize{20}{20}\selectfont \textcolor{CSCoverText}{EUROPE}}
\end{picture}

} 



%-- Summary --------------------------------------------

\newcommand{\executivesummary}{
  \addcontentsline{toc}{chapter}{Executive summary}
  {\huge\headingfont{Executive Summary}}\\[10mm]
  Title: \@delno \@finaltitle \ \\[2mm]
  Subtitle: \@finalsubtitle \ \\[2mm]
  Author: \@finalauthor \ \\[2mm]
  Reference: \@reference\ v \@version\\[2mm]
  Date: \@date\\[2mm]
  Dissemination level: \@dissemination\\[2mm]
  URL: \@URL \\[10mm]
}

%-- Empty Page --------------------------------------------

\newcommand{\emptypage}{
	
 \newpage
 {\color{gray}This page is intentionally blank}
 \pagestyle{fancy}
}

%-- Appendices --------------------------------------------

\newcommand{\appendices}{
\appendix
\titleformat
{\chapter} % command
%[display] % shape
{\huge\headingfont} % format
{\color{CSOrange}\rule{6mm}{6mm}\color{CSRussianGray}\quad \thechapter:} % label
{0.5ex} % sep
{
} % before-code
[
] % after-code

\cleardoublepage
\thispagestyle{empty}
\vspace*{\fill}
{\hfill\huge\headingfont{Appendices}}
\vspace*{\fill}

}



%-- Versions Page --------------------------------------------

\newcommand{\documentversions}{
\thispagestyle{fancy}

\newpage
\addcontentsline{toc}{chapter}{Document Versions}
{\huge\headingfont{Document Versions}}\\[10mm]
}

\newcommand{\versionitem}[4]{
\begin{tabular}[t]{R{2cm}L{14cm}} 
	\bkx \textbf{Version} & \bkx  \textbf{#1} | \textbf{#4} \\
	\emph{Changes}   & #2  \\
	\emph{Authors}   & #3 \\[4mm]
\end{tabular}
\newline
}

%-- Approval --------------------------------------------

\newcommand{\approval}[6]{

\thispagestyle{fancy}
\vfill
\section*{Deliverable Approval}

#2 --- \textbf{WPL}: #1\\ 
#4 --- \textbf{PC}:  #3\\
#6 --- \textbf{QA}:  #5\\

 \pagestyle{fancy}
}


%-- Float Placement Restrictions -----------------------------

\renewcommand{\textfraction}{0.05}
\renewcommand{\topfraction}{0.95}
\renewcommand{\bottomfraction}{0.95}

%-- Headers & Footers -------------------------------------------

\pagestyle{fancy}
\fancyhf{}                                                                           

\fancyheadoffset[LE]{0cm}
\fancyheadoffset[RO]{0cm}

\fancyfootoffset[RO]{0cm}
\fancyfootoffset[LE]{0cm}

\fancyfoot[LE]{
   \small\textsf{\thepage-\pageref{LastPage} \textbar\ CORESENSE\ \@delno\ 
   \textbar\ \textbf{\@reference} \textbar\ \@version\ \textbar\ \@date}
}

\fancyfoot[RO]{
   \small\textsf{CORESENSE\ \@delno \textbar\ \textbf{\@reference} \textbar\ 
   \@version\ \textbar\ \@date\ \textbar\ \thepage-\pageref{LastPage}}
}

\fancyfoot[LO,RE]{}
\fancyhead[LO,RE]{}
\fancyhead[RO]{\nouppercase{\small\textsf{\rightmark}}}
\fancyhead[LE]{\nouppercase{\small\textsf{\leftmark}}}

\renewcommand{\headrulewidth}{0pt}
\renewcommand{\footrulewidth}{0pt}


%-- Page Geometry ------------------------------------------------------------

\newcommand\setnormalpage{
\newgeometry{top=25mm, left=25mm, bottom=30mm, right=25mm} 
\parindent=0 true mm
\parskip=2 true mm
\pagestyle{fancy}
\setlength{\headheight}{14pt}
}

\setnormalpage

%%
%% End of file `csreport.cls'.
