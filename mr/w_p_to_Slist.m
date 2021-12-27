function Slist = w_p_to_Slist(w,p,J)
    Slist = []
    for i = 1:1:J
        w_ = w(i,:);
        p_ = p(i,:);
        v_ = -cross(w_,p_);
        Slist = [Slist,[w_,v_]']
    end
end
