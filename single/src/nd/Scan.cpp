#include <stdio.h>
#include "TData.h"
#include "calcul.h"
#include "Scan.h"
#include <math.h>
#include "geometria.h"

#define pow2(x) (x)*(x)

/* #define MHOUGHp 10 */
/* #define MHOUGHt 10 */

typedef struct {
        float A, B, C;
} Tcoef;

typedef struct {
        int numLines;
        Tcoef lines[361];
} TLines;

typedef struct {
        float rx, ry, nx, ny, dist;
} TAsoc;

void InitSM(TSMparams *params) {
    params->Bw = PI / 40; /* Angular region to search for correspondences */
    params->Br = 0.09; /* Distance threshold for outliers (square value) */
    //  params->error_th=0.001;	  /* error ratio threshold to stop iterating */
    //params->error_r=0.005;
    params->error_th = 0.002; /* error ratio threshold to stop iterating */
    params->error_r = 0.001;
    params->MaxIter = 25;
}

static void motionLMS(TAsoc *in, int numAsoc, Tsc *out) {
    /* % LMS solution for rotation and translation */
    /* % of pairs [ (x,y) <-> (xp,yp)] */
    /* % all input should have [n,1] dimension */

    float ux, uy, uxp, uyp, Sxx, Syy, Sxy, Syx;
    float a, b, c, d;

    float sini, cosi;

    ux = 0;
    uy = 0;
    uxp = 0;
    uyp = 0;
    for (int i = 0; i < numAsoc; i++) {
        ux = ux + in[i].nx;
        uy = uy + in[i].ny;
        uxp = uxp + in[i].rx;
        uyp = uyp + in[i].ry;
    }
    ux = ux / numAsoc;
    uy = uy / numAsoc;
    uxp = uxp / numAsoc;
    uyp = uyp / numAsoc;

    Sxx = 0;
    Syx = 0;
    Sxy = 0;
    Syy = 0;
    for (int i = 0; i < numAsoc; i++) {
        a = in[i].nx - ux;
        b = in[i].ny - uy;
        c = in[i].rx - uxp;
        d = in[i].ry - uyp;

        Sxx = Sxx + (a) * (c);
        Sxy = Sxy + (a) * (d);
        Syx = Syx + (b) * (c);
        Syy = Syy + (b) * (d);
    }

    out->tita = atan((Sxy - Syx) / (Sxx + Syy));
    cosi = cos(out->tita);
    sini = sin(out->tita);
    out->x = uxp - (ux * cosi - uy * sini);
    out->y = uyp - (ux * sini + uy * cosi);
}

static void computeLines(Tscan *puntos, TLines *lin) {

    float dx, dy, d;

    for (int i = 1; i < puntos->numPuntos; i++) {
        if (puntos->laserC[i].x == puntos->laserC[i - 1].x) {
            lin->lines[i - 1].A = 1;
            lin->lines[i - 1].B = 0;
            lin->lines[i - 1].C = -puntos->laserC[i].x;
        } else {
            dx = puntos->laserC[i].x - puntos->laserC[i - 1].x;
            dy = puntos->laserC[i].y - puntos->laserC[i - 1].y;
            d = dy / dx;

            lin->lines[i - 1].A = d;
            lin->lines[i - 1].B = -1;
            lin->lines[i - 1].C = puntos->laserC[i].y - (d
                    * puntos->laserC[i].x);
        }
    }
    lin->numLines = puntos->numPuntos - 1;
}

int scanMatchingLumilios(Tsc *sistemaSalida, Tscan *ptosNew, Tscan *ptosRef,
        Tsc *estimacion, TSMparams *params) {

    Tsc solucion;
    static Tscan ptosNewRef;
    static TLines lin;
    static TAsoc cp_associations[361];
    static TAsoc mr_associations[361];
    /*   static TAsoc cp_associations2[361]; */
    /*   static TAsoc mr_associations2[361]; */

    int cntAssociationsR = 0;
    int cntAssociationsT = 0;
    int cntAssociations = 0;
    Tsc estim_mr;
    Tsc estim_cp;
    int iter, L, R, Io;
    float dist;
    float cp_ass_ptX, cp_ass_ptY, cp_ass_ptD;
    Tpfp tmp_mr_ind;
    float theta_int, dist_k1, interx, intery, a1, b1, c1, a2, b2, c2, den;
    float tmp_cp_indX, tmp_cp_indY, tmp_cp_indD, error1, error2;
    Tpfp mr_ass_pt;
    Tpf mr;
    float mediaR, mediaT;

    for (int i = 0; i < ptosRef->numPuntos; i++)
        car2pol(&ptosRef->laserC[i], &ptosRef->laserP[i]);

    /* Initialise solution with the a priori estimation */
    solucion.x = estimacion->x;
    solucion.y = estimacion->y;
    solucion.tita = estimacion->tita;

    ptosNewRef.numPuntos = ptosNew->numPuntos;
    for (int i = 0; i < ptosNew->numPuntos; i++) {
        transfor_directa_p(ptosNew->laserC[i].x, ptosNew->laserC[i].y,
                estimacion, &ptosNewRef.laserC[i]);
        car2pol(&ptosNewRef.laserC[i], &ptosNewRef.laserP[i]); /** ojo **/
    }

    /* Filtro de proyeccion */
    /* elimina los puntos que no puedes ver */
    /* Ademas ordena los datos segun el angulo */
    int cnt = 1; /* ojo con la inicializacion del filtro a der o izq (si rot>90) */
    for (int i = 1; i < ptosNew->numPuntos; i++) {
        if (ptosNewRef.laserP[i].t >= ptosNewRef.laserP[i - 1].t) { /* punto visible */
            ptosNewRef.laserP[cnt] = ptosNewRef.laserP[i];
            ptosNewRef.laserC[cnt] = ptosNewRef.laserC[i];
            cnt++;
        }
    }
    ptosNewRef.numPuntos = cnt;

    /* TBI filtrar para puntos muy cercanos */ /* !!!! Ayudara !!!*/
     /* TBI filtro de resampleado */

    /* Creamos las lineas entre puntos */
    computeLines(ptosRef, &lin);

    /* Bucle de iteracion hasta convergencia */
    iter = 0;
    while (iter <= params->MaxIter) {
        /*     printf("%d\n"); */
        /* Alineamos indices */
        L = 0;
        R = 0; /* indices de la ventana para recorrer ptoRef */
        Io = 0; /* indices  para recorrer ptoNewRef */

        if (ptosNewRef.laserP[Io].t < ptosRef->laserP[L].t) /* elimino de los nuevos */
            if (ptosNewRef.laserP[Io].t + params->Bw < ptosRef->laserP[L].t)
                while (Io < ptosNewRef.numPuntos - 1 && ptosNewRef.laserP[Io].t
                        + params->Bw < ptosRef->laserP[L].t)
                    Io++;
            else
                while (R < ptosRef->numPuntos - 1 && ptosNewRef.laserP[Io].t
                        + params->Bw > ptosRef->laserP[R + 1].t)
                    R++;
        else {
            while (L < ptosRef->numPuntos - 1 && ptosNewRef.laserP[Io].t
                    - params->Bw > ptosRef->laserP[L].t)
                L++;
            R = L;
            while (R < ptosRef->numPuntos - 1 && ptosNewRef.laserP[Io].t
                    + params->Bw > ptosRef->laserP[R + 1].t)
                R++;
        }

        /**********************************/
        /* Look for correspondences */
        /**********************************/
        cnt = 0;
        for (int i = Io; i < ptosNewRef.numPuntos; i++) {
            while (L < ptosRef->numPuntos - 1 && ptosNewRef.laserP[i].t
                    - params->Bw > ptosRef->laserP[L].t)
                L = L + 1;
            while (R < ptosRef->numPuntos - 1 && ptosNewRef.laserP[i].t
                    + params->Bw > ptosRef->laserP[R + 1].t)
                R = R + 1;

            if (L == R) {
                dist = (ptosNewRef.laserC[i].x - ptosRef->laserC[R].x)
                        * (ptosNewRef.laserC[i].x - ptosRef->laserC[R].x)
                        + (ptosNewRef.laserC[i].y - ptosRef->laserC[R].y)
                                * (ptosNewRef.laserC[i].y
                                        - ptosRef->laserC[R].y);

                if (dist < params->Br) {

                    mr_associations[cnt].nx = ptosNewRef.laserC[i].x;
                    mr_associations[cnt].ny = ptosNewRef.laserC[i].y;
                    mr_associations[cnt].rx = ptosRef->laserC[R].x;
                    mr_associations[cnt].ry = ptosRef->laserC[R].y;
                    mr_associations[cnt].dist = dist;

                    cp_associations[cnt].nx = ptosNewRef.laserC[i].x;
                    cp_associations[cnt].ny = ptosNewRef.laserC[i].y;
                    cp_associations[cnt].rx = ptosRef->laserC[R].x;
                    cp_associations[cnt].ry = ptosRef->laserC[R].y;
                    cp_associations[cnt].dist = dist;

                    cnt++;
                }
            }

            else if (L < R) {
                mr_ass_pt.t = 0;
                mr_ass_pt.r = 100000;/*  % [theta ro] */
                cp_ass_ptX = 0;
                cp_ass_ptY = 0;
                cp_ass_ptD = 100000;

                /* Look for the associated point */
                /* ------------------------------ */
                /* Matching-Range rule */
                for (int J = L + 1; J <= R; J++) {
                    if ((ptosRef->laserP[J - 1].r <= ptosNewRef.laserP[i].r
                            && ptosRef->laserP[J].r <= ptosNewRef.laserP[i].r)
                            || (ptosRef->laserP[J - 1].r
                                    >= ptosNewRef.laserP[i].r
                                    && ptosRef->laserP[J].r
                                            >= ptosNewRef.laserP[i].r))

                        if (fabs(ptosRef->laserP[J - 1].r
                                - ptosNewRef.laserP[i].r) < fabs(
                                ptosRef->laserP[J].r - ptosNewRef.laserP[i].r))
                            tmp_mr_ind = ptosRef->laserP[J - 1];
                        else
                            tmp_mr_ind = ptosRef->laserP[J];
                    else {
                        theta_int = (ptosNewRef.laserP[i].r
                                * (ptosRef->laserP[J - 1].r * ptosRef->laserP[J
                                        - 1].t - ptosRef->laserP[J].r
                                        * ptosRef->laserP[J].t)
                                + ptosRef->laserP[J - 1].r
                                        * ptosRef->laserP[J].r
                                        * (ptosRef->laserP[J].t
                                                - ptosRef->laserP[J - 1].t))
                                / (ptosNewRef.laserP[i].r * (ptosRef->laserP[J
                                        - 1].r - ptosRef->laserP[J].r));
                        tmp_mr_ind.t = theta_int;
                        tmp_mr_ind.r = ptosNewRef.laserP[i].r;
                    }

                    if ((fabs(tmp_mr_ind.r - ptosNewRef.laserP[i].r) < fabs(
                            mr_ass_pt.r - ptosNewRef.laserP[i].r))
                            || (((int) (0.5F + 1000.0F * fabs(tmp_mr_ind.r
                                    - ptosNewRef.laserP[i].r)) == (int) (0.5F
                                    + 1000.0F * fabs(mr_ass_pt.r
                                            - ptosNewRef.laserP[i].r)))
                                    && (fabs(tmp_mr_ind.t
                                            - ptosNewRef.laserP[i].t) < fabs(
                                            mr_ass_pt.t
                                                    - ptosNewRef.laserP[i].t)))) {
                        mr_ass_pt.t = tmp_mr_ind.t;
                        mr_ass_pt.r = tmp_mr_ind.r;
                    }
                }

                dist_k1 = pow2(ptosNewRef.laserC[i].x-ptosRef->laserC[L].x)
                        + pow2(ptosNewRef.laserC[i].y-ptosRef->laserC[L].y);

                /* Closest point rule */
                for (int J = L + 1; J <= R; J++) {
                    dist
                            = pow2(ptosNewRef.laserC[i].x-ptosRef->laserC[J].x)
                                    + pow2(ptosNewRef.laserC[i].y-ptosRef->laserC[J].y);
                    if (lin.lines[J - 1].B == 0) {
                        interx = ptosRef->laserC[J].x;
                        intery = ptosNewRef.laserC[i].y;
                    } else {
                        a1 = lin.lines[J - 1].A;
                        b1 = lin.lines[J - 1].B;
                        c1 = lin.lines[J - 1].C;
                        if (ptosRef->laserC[J].y == ptosRef->laserC[J - 1].y) {
                            a2 = 1;
                            b2 = 0;
                            c2 = ptosRef->laserC[J].y;
                            interx = ptosNewRef.laserC[i].x;
                            intery = ptosRef->laserC[J].y;
                        } else {
                            a2 = -1 / a1;
                            b2 = -1;
                            c2 = -a2 * ptosNewRef.laserC[i].x
                                    + ptosNewRef.laserC[i].y;
                            den = -a1 + a2;
                            interx = (-c2 + c1) / den;
                            intery = (c1 * a2 - c2 * a1) / den;
                        }
                    }
                    if (((interx < ptosRef->laserC[J].x && interx
                            > ptosRef->laserC[J - 1].x) || (interx
                            > ptosRef->laserC[J].x && interx
                            < ptosRef->laserC[J - 1].x)) && ((intery
                            < ptosRef->laserC[J].y && intery
                            > ptosRef->laserC[J - 1].y) || (intery
                            > ptosRef->laserC[J].y && intery
                            < ptosRef->laserC[J - 1].y))) {
                        /* take the intersection point between the two straight lines */
                        dist = pow2(ptosNewRef.laserC[i].x-interx)
                                + pow2(ptosNewRef.laserC[i].y-intery);
                        tmp_cp_indX = interx;
                        tmp_cp_indY = intery;
                        tmp_cp_indD = dist;
                    } else {
                        /* take the nearest end-point of the segment */
                        if (dist < dist_k1) {
                            tmp_cp_indX = ptosRef->laserC[J].x;
                            tmp_cp_indY = ptosRef->laserC[J].y;
                            tmp_cp_indD = dist;
                        } else {
                            tmp_cp_indX = ptosRef->laserC[J - 1].x;
                            tmp_cp_indY = ptosRef->laserC[J - 1].y;
                            tmp_cp_indD = dist_k1;
                        }
                    }

                    if (tmp_cp_indD < cp_ass_ptD) {
                        cp_ass_ptX = tmp_cp_indX;
                        cp_ass_ptY = tmp_cp_indY;
                        cp_ass_ptD = tmp_cp_indD;
                    }
                }

                pol2car(&mr_ass_pt, &mr);

                dist = pow2(ptosNewRef.laserC[i].x-mr.x)
                        + pow2(ptosNewRef.laserC[i].y-mr.y);

                if (dist < params->Br || cp_ass_ptD < params->Br) {

                    mr_associations[cnt].nx = ptosNewRef.laserC[i].x;
                    mr_associations[cnt].ny = ptosNewRef.laserC[i].y;
                    mr_associations[cnt].rx = mr.x;
                    mr_associations[cnt].ry = mr.y;
                    mr_associations[cnt].dist = dist;

                    cp_associations[cnt].nx = ptosNewRef.laserC[i].x;
                    cp_associations[cnt].ny = ptosNewRef.laserC[i].y;
                    cp_associations[cnt].rx = cp_ass_ptX;
                    cp_associations[cnt].ry = cp_ass_ptY;
                    cp_associations[cnt].dist = cp_ass_ptD;

                    cnt++;

                }
            }
        }

        cntAssociations = cnt;

        /* --------------------- */
        /* ----- Resampling ---- */
        /* --------------------- */
        mediaT = 0;
        mediaR = 0;
        for (int i = 0; i < cntAssociations; i++) {
            mediaT = mediaT + cp_associations[i].dist;
            mediaR = mediaR + mr_associations[i].dist;
        }
        mediaT = mediaT / cntAssociations;
        mediaR = mediaR / cntAssociations;

        cntAssociationsT = 0;
        cntAssociationsR = 0;
        mediaT = mediaT * 3 / 2; // ojo !!
        mediaR = mediaR * 3 / 2;
        for (int i = 0; i < cntAssociations; i++) {
            if (cp_associations[i].dist < mediaT) {
                cp_associations[cntAssociationsT] = cp_associations[i];
                cntAssociationsT++;
            }
            if (mr_associations[i].dist < mediaR) {
                mr_associations[cntAssociationsR] = mr_associations[i];
                cntAssociationsR++;
            }
        }

        //     printf("<asocT,asocR> = <%d,%d>\n",cntAssociationsT,cntAssociationsR);

        /* --------------------- */
        /* ---------MINIMOS ------------ */
        /* --------------------- */
        /*     motionLMS(cp_associations2,cntAssociationsT,&estim_cp); */
        /*     motionLMS(mr_associations2,cntAssociationsR,&estim_mr); */

        motionLMS(cp_associations, cntAssociationsT, &estim_cp);
        motionLMS(mr_associations, cntAssociationsR, &estim_mr);

        error1 = fabs(estim_cp.x) + fabs(estim_cp.y);
        error2 = fabs(estim_mr.tita);

        //printf("<eT,eR>=<%f,%f>\n",error1,error2);

        if (error1 < params->error_r && error2 < params->error_th)
            break;

        solucion.x = solucion.x + estim_cp.x;
        solucion.y = solucion.y + estim_cp.y;
        solucion.tita = solucion.tita + estim_mr.tita;

        for (int i = 0; i < ptosNew->numPuntos; i++) {
            transfor_directa_p(ptosNew->laserC[i].x, ptosNew->laserC[i].y,
                    &solucion, &ptosNewRef.laserC[i]);
            car2pol(&ptosNewRef.laserC[i], &ptosNewRef.laserP[i]); /** ojo **/
        }

        iter++;

    }

    *sistemaSalida = solucion;
    if (iter < params->MaxIter)
        return 1;
    else {
        /*     printf("Ey! Er=<%f,%f>\n",error1,error2); */
        return 0;
    }
}
